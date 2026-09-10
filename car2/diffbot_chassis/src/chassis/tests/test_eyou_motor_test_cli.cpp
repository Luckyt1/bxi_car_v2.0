#include "chassis/bxi_pci_transport.hpp"

#include <algorithm>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

struct FakeDrive
{
  std::map<std::uint16_t, std::uint32_t> values{
    {0x6041, 0x40}, {0x603F, 0}, {0x6061, 8}, {0x606C, 0}, {0x2100, 1},
    {0x2025, 131072}, {0x26A2, 101}, {0x26A3, 1}, {0x60FF, 0},
    {0x6064, static_cast<std::uint32_t>(-123456)},
    {0x6079, 48000}, {0x6075, 5000}, {0x6078, static_cast<std::uint32_t>(-250)},
    {0x6076, 2000}, {0x6077, static_cast<std::uint32_t>(-500)},
    {0x2779, 35}, {0x277A, 40}, {0x27BD, 0}};
  std::vector<std::int32_t> target_writes;
  std::vector<std::uint16_t> controls;
  std::vector<bool> power;
  std::set<std::uint16_t> abort_uploads;
  int actual_velocity_reads{0};
  int voltage_reads{0};
  int raise_signal_after_actual_reads{0};
  int signal_to_raise{SIGINT};

  void reset()
  {
    *this = FakeDrive{};
  }
};

FakeDrive fake;

std::uint32_t payload_u32(const iswv::CanFrame & frame)
{
  std::uint32_t value = 0;
  for (unsigned i = 0; i < 4; ++i) {
    value |= std::uint32_t(frame.data[i + 4]) << (i * 8);
  }
  return value;
}

void write_u32(iswv::CanFrame & frame, std::uint32_t value)
{
  for (unsigned i = 0; i < 4; ++i) {
    frame.data[i + 4] = static_cast<std::uint8_t>((value >> (i * 8)) & 0xFF);
  }
}
}  // namespace

namespace chassis
{
BxiPciTransport::BxiPciTransport(unsigned int bus, bool trace)
: bus_(bus), trace_(trace)
{
  require(!trace_, "normal motion output must not enable CAN tracing");
  open_.store(true);
}

BxiPciTransport::~BxiPciTransport()
{
  open_.store(false);
  fake.power.push_back(false);
}

iswv::Result<void> BxiPciTransport::send(const iswv::CanFrame & request)
{
  if (!open_.load()) {
    return iswv::Result<void>::failure(
      iswv::ErrorCode::transport_closed, "fake BXI PCI CAN interface is closed");
  }
  if (request.id == 0) {return iswv::Result<void>::success();}
  if (request.id != 0x601) {
    return iswv::Result<void>::failure(iswv::ErrorCode::transport_error, "unexpected CAN id");
  }

  const auto index = static_cast<std::uint16_t>(request.data[1] | (request.data[2] << 8));
  iswv::CanFrame response;
  response.id = 0x581;
  response.size = 8;
  response.data[1] = request.data[1];
  response.data[2] = request.data[2];
  response.data[3] = request.data[3];

  if (request.data[0] == 0x40) {
    if (index == 0x6079) {++fake.voltage_reads;}
    if (index == 0x606C) {
      ++fake.actual_velocity_reads;
      if (fake.raise_signal_after_actual_reads > 0 &&
        fake.actual_velocity_reads >= fake.raise_signal_after_actual_reads)
      {
        std::raise(fake.signal_to_raise);
      }
    }
    if (fake.abort_uploads.count(index) != 0 && fake.actual_velocity_reads >= 3) {
      response.data[0] = 0x80;
      write_u32(response, 0x06020000);
    } else {
      response.data[0] = (index == 0x6061 || index == 0x2100) ? 0x4F :
        ((index == 0x6041 || index == 0x603F || index == 0x26A2 || index == 0x26A3 ||
        index == 0x6077 || index == 0x6078) ?
        0x4B : 0x43);
      write_u32(response, fake.values[index]);
    }
  } else {
    const auto value = payload_u32(request);
    response.data[0] = 0x60;
    if (index == 0x6040) {
      const auto control = static_cast<std::uint16_t>(value);
      fake.controls.push_back(control);
      if (control == 0) {fake.values[0x6041] = 0x40;}
      if (control == 6 && fake.values[0x6041] == 0x40) {fake.values[0x6041] = 0x21;}
      if (control == 7 && fake.values[0x6041] == 0x21) {fake.values[0x6041] = 0x23;}
      if (control == 15 && fake.values[0x6041] == 0x23) {fake.values[0x6041] = 0x27;}
    } else if (index == 0x2100) {
      fake.values[index] = value;
    } else if (index == 0x6060) {
      fake.values[0x6061] = value;
    } else if (index == 0x60FF) {
      const auto target = static_cast<std::int32_t>(value);
      fake.target_writes.push_back(target);
      fake.values[0x60FF] = value;
      fake.values[0x606C] = value;
    } else {
      fake.values[index] = value;
    }
  }

  iswv::ReceiveHandler handler;
  {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    handler = receive_handler_;
  }
  if (handler) {handler(response);}
  return iswv::Result<void>::success();
}

void BxiPciTransport::set_receive_handler(iswv::ReceiveHandler handler)
{
  std::lock_guard<std::mutex> lock(handler_mutex_);
  receive_handler_ = std::move(handler);
}

void BxiPciTransport::set_error_handler(iswv::TransportErrorHandler handler)
{
  std::lock_guard<std::mutex> lock(handler_mutex_);
  error_handler_ = std::move(handler);
}

bool BxiPciTransport::is_open() const noexcept
{
  return open_.load();
}

std::string BxiPciTransport::name() const
{
  return "fake-bxi-pci:bus" + std::to_string(bus_);
}

iswv::Result<void> BxiPciTransport::set_motor_power(bool enabled)
{
  fake.power.push_back(enabled);
  return iswv::Result<void>::success();
}
}  // namespace chassis

#define main eyou_motor_test_main
#include "../src/eyou_motor_test.cpp"
#undef main

namespace
{
int run_cli(std::vector<std::string> args, double expected_position_rad = -0.058595)
{
  interrupted = 0;
  std::vector<char *> argv;
  argv.reserve(args.size());
  for (auto & arg : args) {
    argv.push_back(arg.data());
  }
  std::ostringstream output;
  auto * previous = std::cout.rdbuf(output.rdbuf());
  const auto result = eyou_motor_test_main(static_cast<int>(argv.size()), argv.data());
  std::cout.rdbuf(previous);
  std::istringstream lines(output.str());
  std::string line;
  unsigned samples = 0;
  while (std::getline(lines, line)) {
    std::istringstream fields(line);
    std::string speed_label, rpm_unit, position_label, position_unit, extra;
    double rpm = 0;
    double position = 0;
    require(
      static_cast<bool>(fields >> speed_label >> rpm >> rpm_unit >> position_label >>
      position >> position_unit), "each normal line must contain speed and position");
    require(
      speed_label == "转速:" && rpm_unit == "RPM" && position_label == "位置:" &&
      position_unit == "rad", "normal output must show motion telemetry");
    std::getline(fields, extra);
    require(extra.find("电压: 48.000 V") != std::string::npos, "must append bus voltage");
    require(extra.find("电流: -1.250 A") != std::string::npos, "must append signed current");
    require(extra.find("电机力矩: -1.000 Nm") != std::string::npos, "must append motor torque");
    require(extra.find("内部温度:") != std::string::npos, "must append internal temperature");
    require(extra.find("功率温度:") != std::string::npos, "must append power temperature");
    require(extra.find("力控力矩:") != std::string::npos, "must append force torque");
    require(std::abs(rpm - 1.0) < 0.001, "feedback must show actual output RPM");
    require(
      std::abs(position - expected_position_rad) < 0.000001,
      "position must convert signed 0x6064 feedback to output radians");
    ++samples;
  }
  require(samples > 0, "motion must produce telemetry before stopping");
  return result;
}

void require_stopped_and_disabled()
{
  require(
    std::any_of(
      fake.target_writes.begin(), fake.target_writes.end(),
      [](std::int32_t speed) {return speed != 0;}), "test must command nonzero velocity");
  require(fake.target_writes.back() == 0, "test must stop with zero velocity");
  require(!fake.controls.empty() && fake.controls.back() == 0, "test must disable voltage");
  require(fake.power == std::vector<bool>({true, false}), "test must power on then off");
}

void continuous_stops_on_signal(int signal_number)
{
  fake.reset();
  fake.signal_to_raise = signal_number;
  fake.raise_signal_after_actual_reads = 3;
  const int code = run_cli({"eyou_motor_test", "1", "1.0", "--continuous"});
  require(code == 130, "continuous signal stop must return 130");
  require(fake.actual_velocity_reads >= 3, "continuous mode must keep polling actual velocity");
  require_stopped_and_disabled();
}
}  // namespace

int main()
{
  try {
    continuous_stops_on_signal(SIGINT);
    continuous_stops_on_signal(SIGTERM);

    fake.reset();
    fake.values[0x2025] = 524288;
    fake.values[0x6064] = 52953088;
    const int timed_code = run_cli({"eyou_motor_test", "1", "1.0", "1"}, 6.283185);
    require(timed_code == 0, "legacy timed run must still exit successfully");
    require(fake.voltage_reads == 1, "electrical telemetry must be cached between 1 s updates");
    require(fake.actual_velocity_reads > 0, "timed run must poll actual velocity");
    require_stopped_and_disabled();

    fake.reset();
    fake.abort_uploads.insert(0x606C);
    const int failed_code = run_cli({"eyou_motor_test", "1", "1.0", "--continuous"});
    require(failed_code == 1, "communication failure must exit with error");
    require_stopped_and_disabled();

    fake.reset();
    fake.abort_uploads.insert(0x6064);
    const int position_failed_code = run_cli({"eyou_motor_test", "1", "1.0", "--continuous"});
    require(position_failed_code == 1, "position read failure must exit with error");
    require_stopped_and_disabled();
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
