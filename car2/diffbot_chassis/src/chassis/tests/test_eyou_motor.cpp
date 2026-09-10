#include "chassis/eyou_motor.hpp"
#include "iswv/fake_transport.hpp"

#include <chrono>
#include <iostream>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

// In-process SDO server: verify payload widths, signed speed, and CiA402 order.
struct Servo
{
  std::shared_ptr<iswv::FakeTransport> bus = std::make_shared<iswv::FakeTransport>();
  std::map<std::uint16_t, std::uint32_t> values{
    {0x6041, 0x40}, {0x6061, 8}, {0x606C, 0}, {0x2100, 1},
    {0x2025, 131072}, {0x26A2, 101}, {0x26A3, 1},
    {0x6079, 48000}, {0x6075, 2500}, {0x6078, static_cast<std::uint32_t>(-500)},
    {0x6076, 2000}, {0x6077, static_cast<std::uint32_t>(-500)}, {0x2779, 42},
    {0x277A, 55}, {0x27BD, static_cast<std::uint32_t>(-7)}};
  std::vector<std::uint16_t> controls;
  std::vector<std::uint16_t> downloads;
  bool reject_acceleration{false};
  bool reject_zero{false};
  bool reject_reads{false};
  bool drop_reads{false};
  bool wrong_mode{false};
  unsigned delayed_mode_reads{0};
  unsigned pending_mode_reads{0};
  unsigned mode_reads{0};
  bool enabled_before_mode_confirmed{false};
  bool diagnostic_errors{false};
  bool reject_control_source{false};
  bool ignore_control_source{false};
  std::set<std::uint16_t> reject_read_indices;
  std::set<std::uint16_t> drop_read_indices;
  std::uint16_t bad_width{0};

  Servo()
  {
    bus->set_send_hook(
      [this](const iswv::CanFrame & request) {
        if (request.id != 0x601) {return;}
        const auto index = static_cast<std::uint16_t>(request.data[1] | (request.data[2] << 8));
        if (drop_reads && index == 0x606C) {return;}
        if (request.data[0] == 0x40 && drop_read_indices.count(index) != 0) {return;}
        if (diagnostic_errors && index == 0x6041 && request.data[0] == 0x40) {return;}
        iswv::CanFrame response;
        response.id = 0x581;
        response.size = 8;
        response.data[1] = request.data[1];
        response.data[2] = request.data[2];
        response.data[3] = request.data[3];
        if (request.data[0] != 0x40) {
          downloads.push_back(index);
          if ((values[0x2100] != 2 &&
          (index == 0x6040 || index == 0x60FF || index == 0x6060)) ||
          (reject_control_source && index == 0x2100))
          {
            response.data[0] = 0x80;
            response.data[4] = 2;
            bus->inject(response);
            return;
          }
        }
        std::uint32_t value = 0;
        for (unsigned i = 0; i < 4; ++i) {value |= std::uint32_t(request.data[i + 4]) << (i * 8);}
        if (request.data[0] == 0x40) {
          if (index == 0x6061) {
            ++mode_reads;
            if (pending_mode_reads > 0 && --pending_mode_reads == 0) {values[0x6061] = 3;}
          }
          value = values[index];
          response.data[0] = (index == 0x6041 || index == 0x603F ||
          index == 0x26A2 || index == 0x26A3 || index == 0x6078 || index == 0x6077) ?
          0x4B : ((index == 0x6061 || index == 0x2100) ? 0x4F : 0x43);
        } else {
          const auto expected = index == 0x6040 ?
          0x2B : ((index == 0x6060 || index == 0x2100) ? 0x2F : 0x23);
          if (request.data[0] != expected) {bad_width = index;}
          if (index == 0x6040) {
            if (value == 15 && values[0x6061] != 3) {enabled_before_mode_confirmed = true;}
            controls.push_back(static_cast<std::uint16_t>(value));
            if (value == 0) {values[0x6041] = 0x40;}
            if (value == 6 && values[0x6041] == 0x40) {values[0x6041] = 0x21;}
            if (value == 7 && values[0x6041] == 0x21) {values[0x6041] = 0x23;}
            if (value == 15 && values[0x6041] == 0x23) {values[0x6041] = 0x27;}
          }
          if (!(ignore_control_source && index == 0x2100)) {values[index] = value;}
          if (index == 0x6060) {
            pending_mode_reads = wrong_mode ? 0 : delayed_mode_reads;
            mode_reads = 0;
            values[0x6061] = wrong_mode ? 9 : (pending_mode_reads > 0 ? 8 : value);
          }
          if (index == 0x60FF) {values[0x606C] = value;}
          response.data[0] = 0x60;
        }
        if ((reject_acceleration && index == 0x6083) ||
        (reject_zero && index == 0x60FF && value == 0) ||
        (reject_reads && index == 0x606C) ||
        (request.data[0] == 0x40 && reject_read_indices.count(index) != 0))
        {
          response.data[0] = 0x80;
          value = 0x06020000;
        }
        if (diagnostic_errors && index == 0x603F) {
          response.data[0] = 0x80;
          value = 2;
        }
        for (unsigned i = 0; i < 4; ++i) {response.data[i + 4] = (value >> (i * 8)) & 0xFF;}
        bus->inject(response);
      });
  }
};

template<typename F>
void must_fail(F action)
{
  bool failed = false;
  try {
    action();
  } catch (const std::exception &) {
    failed = true;
  }
  require(failed, "expected failure");
}
}  // namespace

int main()
{
  try {
    Servo servo;
    iswv::canopen::CanopenMaster master(servo.bus);
    chassis::EyouMotor motor(master, 1);
    must_fail([&] {motor.set_velocity(1000);});
    must_fail([&] {motor.enable(0, 20000);});
    require(servo.bus->sent_frames().empty(), "invalid input must not send");
    servo.values[0x603F] = 0x3220;
    const auto diagnostic = motor.diagnostic_report();
    require(
      diagnostic.find("0x6041:00 Status word = 0x0040") != std::string::npos,
      "diagnostics must show status word");
    require(
      diagnostic.find("0x603f:00 error code = 0x3220") != std::string::npos,
      "diagnostics must show device error code");
    require(
      diagnostic.find("0x6061:00 mode display = 0x08") != std::string::npos,
      "diagnostics must show mode");
    for (const auto & frame : servo.bus->sent_frames()) {
      require(frame.id == 0x601 && frame.data[0] == 0x40, "diagnostics must only read");
    }
    servo.diagnostic_errors = true;
    const auto failed_diagnostic = motor.diagnostic_report();
    require(
      failed_diagnostic.find("READ FAILED") != std::string::npos,
      "diagnostic timeout must be reported");
    require(
      failed_diagnostic.find("SDO abort 0x00000002") != std::string::npos,
      "diagnostic abort must be reported in full");
    require(
      failed_diagnostic.find("mode display = 0x08") != std::string::npos,
      "diagnostics must continue after failed reads");
    servo.diagnostic_errors = false;
    servo.values[0x60FF] = static_cast<std::uint32_t>(-330957);
    servo.values[0x606C] = static_cast<std::uint32_t>(-330957);
    servo.bus->clear_sent();
    {
      chassis::EyouMotor info_motor(master, 1);
      bool complete = false;
      const auto information = info_motor.information_report(&complete);
      require(complete, "complete information report must mark success");
      require(
        information.find("0x6041:00 Status word = 0x0040 (switch on disabled)") !=
        std::string::npos,
        "information must show disabled status");
      require(
        information.find("0x603f:00 error code = 0x3220") != std::string::npos,
        "information must show fault code");
      require(
        information.find("0x6061:00 mode display = 8") != std::string::npos,
        "information must show mode display");
      require(
        information.find("0x2100:00 control source = 1 (EtherCAT)") != std::string::npos,
        "information must show control source");
      require(
        information.find("0x2025:00 encoder counts/revolution = 131072") != std::string::npos,
        "information must show encoder resolution");
      require(
        information.find("0x26a2:00 gear numerator = 101") != std::string::npos,
        "information must show reduction numerator");
      require(
        information.find("0x26a3:00 gear denominator = 1") != std::string::npos,
        "information must show reduction denominator");
      require(
        information.find("0x60ff:00 target velocity = -330957 pulse/s (-1.500 output RPM)") !=
        std::string::npos,
        "information must convert signed target RPM");
      require(
        information.find("0x606c:00 actual velocity = -330957 pulse/s (-1.500 output RPM)") !=
        std::string::npos,
        "information must read disabled signed feedback RPM");
    }
    for (const auto & frame : servo.bus->sent_frames()) {
      require(frame.id == 0x601 && frame.data[0] == 0x40, "information must only send SDO reads");
    }
    servo.bus->clear_sent();
    bool electrical_complete = false;
    const auto electrical = motor.electrical_report(&electrical_complete);
    require(electrical_complete, "complete electrical report must mark success");
    require(
      electrical.find("电压: 48.000 V") != std::string::npos,
      "electrical report must convert DC bus voltage from mV to V");
    require(
      electrical.find("电流: -1.250 A") != std::string::npos,
      "electrical report must convert signed current ratio and rated mA to A");
    require(
      electrical.find("电机力矩: -1.000 Nm") != std::string::npos,
      "electrical report must convert signed torque ratio and rated mNm to Nm");
    require(
      electrical.find("内部温度: 42.000 C") != std::string::npos,
      "electrical report must show internal temperature");
    require(
      electrical.find("功率温度: 55.000 C") != std::string::npos,
      "electrical report must show power module temperature");
    require(
      electrical.find("力控力矩: -7.000 Nm") != std::string::npos,
      "electrical report must show force-control torque");
    require(servo.bus->sent_frames().size() == 8, "electrical report must read eight objects");
    for (const auto & frame : servo.bus->sent_frames()) {
      require(
        frame.id == 0x601 && frame.data[0] == 0x40,
        "electrical report must only send SDO reads");
    }
    servo.reject_read_indices = {0x6079, 0x2779};
    servo.drop_read_indices = {0x277A};
    servo.bus->clear_sent();
    electrical_complete = true;
    const auto started = std::chrono::steady_clock::now();
    const auto partial_electrical = motor.electrical_report(&electrical_complete);
    const auto elapsed = std::chrono::steady_clock::now() - started;
    require(!electrical_complete, "partial electrical report must mark incomplete");
    require(
      elapsed < std::chrono::milliseconds(
        200), "electrical report SDO timeout must be bounded");
    require(
      partial_electrical.find("电压: 不可用") != std::string::npos,
      "electrical report must show unavailable voltage after abort");
    require(
      partial_electrical.find("0x6079") != std::string::npos,
      "electrical voltage failure must identify object");
    require(
      partial_electrical.find("内部温度: 不可用") != std::string::npos,
      "electrical report must show unavailable internal temperature after abort");
    require(
      partial_electrical.find("功率温度: 不可用") != std::string::npos,
      "electrical report must show unavailable power temperature after timeout");
    require(
      partial_electrical.find("电流: -1.250 A") != std::string::npos &&
      partial_electrical.find("力控力矩: -7.000 Nm") != std::string::npos,
      "electrical report must continue after individual read failures");
    require(
      servo.bus->sent_frames().size() == 8,
      "partial electrical report must still try every object");
    for (const auto & frame : servo.bus->sent_frames()) {
      require(
        frame.id == 0x601 && frame.data[0] == 0x40,
        "partial electrical report must only send SDO reads");
    }
    servo.reject_read_indices.clear();
    servo.drop_read_indices.clear();
    servo.values[0x6075] = 0;
    servo.values[0x6076] = 0;
    electrical_complete = true;
    const auto zero_rated_electrical = motor.electrical_report(&electrical_complete);
    require(!electrical_complete, "zero rated values must mark electrical report incomplete");
    require(
      zero_rated_electrical.find("电流: 不可用") != std::string::npos,
      "zero rated current must be unavailable");
    require(
      zero_rated_electrical.find("电机力矩: 不可用") != std::string::npos,
      "zero rated torque must be unavailable");
    servo.values[0x6075] = 2500;
    servo.values[0x6076] = 2000;
    servo.reject_read_indices = {0x2025, 0x606C};
    bool partial_complete = true;
    const auto partial_information = motor.information_report(&partial_complete);
    require(!partial_complete, "partial information report must mark incomplete");
    require(
      partial_information.find("0x60ff:00 target velocity = -330957 pulse/s") !=
      std::string::npos,
      "information must continue after read failures");
    require(
      partial_information.find("0x2025:00 encoder counts/revolution = READ FAILED") !=
      std::string::npos,
      "information must report encoder read failure");
    require(
      partial_information.find("0x606c:00 actual velocity = READ FAILED") != std::string::npos,
      "information must report actual velocity read failure");
    servo.reject_read_indices.clear();
    servo.values[0x26A3] = 0;
    bool invalid_scale_complete = true;
    const auto invalid_scale_information = motor.information_report(&invalid_scale_complete);
    require(!invalid_scale_complete, "invalid ratio must mark information incomplete");
    require(
      invalid_scale_information.find(
        "output RPM unavailable: invalid encoder resolution/reduction ratio") !=
      std::string::npos,
      "invalid ratio must report unavailable RPM");
    servo.values[0x26A3] = 1;
    servo.values[0x60FF] = 0;
    servo.values[0x606C] = 0;
    servo.delayed_mode_reads = 3;
    motor.enable(20000, 20000);
    require(servo.mode_reads == 3, "must wait for delayed mode display");
    require(
      servo.values[0x2100] == 2 && servo.downloads.front() == 0x2100,
      "CANopen authority must be selected before motion writes");
    require(
      diagnostic.find("control source = 0x01 (EtherCAT)") != std::string::npos,
      "diagnostics must identify control source");
    require(servo.controls == std::vector<std::uint16_t>({0, 6, 7, 15}), "enable sequence");
    require(servo.bad_width == 0, "wrong SDO data width");
    motor.set_velocity(-1000);
    require(motor.actual_velocity() == -1000, "signed velocity feedback");
    require(servo.values[0x60FF] == 0xFFFFFC18, "signed little-endian velocity");
    motor.stop();
    require(servo.values[0x60FF] == 0 && servo.controls.back() == 0, "normal stop");
    motor.enable(1000, 1000);
    servo.values[0x6041] = 8;
    must_fail([&] {motor.set_velocity(1000);});
    require(servo.values[0x60FF] == 0, "must not command speed while faulted");
    motor.stop();
    servo.reject_acceleration = true;
    must_fail([&] {motor.enable(1000, 1000);});
    require(servo.controls.back() == 0, "partial initialization must disable");
    servo.reject_acceleration = false;
    servo.wrong_mode = true;
    must_fail([&] {motor.enable(1000, 1000);});
    require(servo.controls.back() == 0, "mode mismatch must disable");
    servo.wrong_mode = false;
    motor.enable(1000, 1000);
    servo.reject_reads = true;
    must_fail([&] {motor.actual_velocity();});
    servo.reject_reads = false;
    servo.drop_reads = true;
    must_fail([&] {motor.actual_velocity();});
    servo.drop_reads = false;
    servo.reject_zero = true;
    must_fail([&] {motor.stop();});
    require(servo.controls.back() == 0, "failed zero-speed command must still disable");
    servo.reject_zero = false;
    {
      chassis::EyouMotor scoped(master, 1);
      scoped.enable(20000, 20000);
      scoped.set_velocity(1000);
    }
    require(servo.values[0x60FF] == 0 && servo.controls.back() == 0, "destructor must stop");
    servo.values[0x2100] = 1;
    servo.values[0x6041] = 0x27;
    servo.downloads.clear();
    must_fail([&] {motor.enable(1000, 1000);});
    motor.stop();
    require(servo.downloads.empty(), "must not take authority from an enabled drive");
    servo.values[0x6041] = 0x40;
    servo.reject_control_source = true;
    must_fail([&] {motor.enable(1000, 1000);});
    motor.stop();
    require(
      servo.downloads == std::vector<std::uint16_t>({0x2100}),
      "rejected authority must prevent motion writes and cleanup writes");
    servo.reject_control_source = false;
    servo.ignore_control_source = true;
    servo.downloads.clear();
    must_fail([&] {motor.enable(1000, 1000);});
    require(
      servo.downloads == std::vector<std::uint16_t>({0x2100}),
      "authority must be read back, not just acknowledged");
    servo.ignore_control_source = false;
    servo.values[0x2100] = 0;
    motor.enable(1000, 1000);
    require(servo.values[0x2100] == 2, "UART authority must also switch to CANopen");
    motor.stop();
    require(!servo.enabled_before_mode_confirmed, "must confirm mode before enabling");
    must_fail([&] {motor.set_velocity_rpm(1);});
    const double pulses_per_rev = motor.configure_rpm_units();
    require(pulses_per_rev == 13238272, "output scale must include encoder counts and reducer");
    require(motor.velocity_from_rpm(20) == 4412757, "20 output RPM conversion");
    require(motor.velocity_from_rpm(-1.5) == -330957, "fractional negative RPM rounding");
    require(motor.velocity_from_rpm(0) == 0, "zero RPM conversion");
    must_fail([&] {motor.velocity_from_rpm(std::numeric_limits<double>::quiet_NaN());});
    must_fail([&] {motor.velocity_from_rpm(std::numeric_limits<double>::infinity());});
    must_fail([&] {motor.velocity_from_rpm(1e100);});
    must_fail([&] {motor.velocity_from_rpm(1e-12);});
    must_fail([&] {motor.enable_rpm(-1, 10);});
    motor.enable_rpm(10, 10);
    require(
      servo.values[0x6083] == 2206379 && servo.values[0x6084] == 2206379,
      "acceleration and deceleration must use the same RPM scale");
    must_fail([&] {motor.configure_rpm_units();});
    motor.set_velocity_rpm(-1.5);
    require(
      std::fabs(motor.actual_velocity_rpm() + 1.5) < 0.00001,
      "feedback must convert back to signed output RPM");
    motor.stop();
    servo.values[0x26A2] = 3;
    servo.values[0x26A3] = 2;
    require(motor.configure_rpm_units() == 196608, "fractional reducer ratio");
    require(motor.velocity_from_rpm(60) == 196608, "60 RPM equals one revolution per second");
    servo.values[0x2025] = 0;
    must_fail([&] {motor.configure_rpm_units();});
    must_fail([&] {motor.velocity_from_rpm(1);});
    servo.values[0x2025] = 131072;
    servo.values[0x26A3] = 0;
    must_fail([&] {motor.configure_rpm_units();});
    std::cout << "Eyou simulated protocol checks passed\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
