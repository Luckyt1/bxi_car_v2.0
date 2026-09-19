#include "chassis/motor/yiyou_motor.h"

#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace chassis
{
namespace
{
using namespace std::chrono_literals;
using iswv::canopen::Object;

constexpr Object<std::int8_t> mode_display{{0x6061, 0}, "mode display"};
constexpr Object<std::int32_t> target_velocity{{0x60FF, 0}, "target velocity"};
constexpr Object<std::int32_t> velocity_actual{{0x606C, 0}, "actual velocity"};
constexpr Object<std::uint32_t> profile_acceleration{{0x6083, 0}, "acceleration"};
constexpr Object<std::uint32_t> profile_deceleration{{0x6084, 0}, "deceleration"};
constexpr Object<std::uint8_t> control_source{{0x2100, 0}, "control source"};
constexpr Object<std::int32_t> encoder_resolution{{0x2025, 0}, "encoder counts/revolution"};
constexpr Object<std::uint16_t> gear_numerator{{0x26A2, 0}, "gear numerator"};
constexpr Object<std::uint16_t> gear_denominator{{0x26A3, 0}, "gear denominator"};

template<typename T>
void check(const iswv::Result<T> & result, const char * operation)
{
  if (result) {return;}
  std::ostringstream message;
  message << operation << ": " << result.error().message;
  if (result.error().sdo_abort_code) {
    message << " (SDO abort 0x" << std::hex << std::setfill('0') << std::setw(8)
            << *result.error().sdo_abort_code << ")";
  }
  throw std::runtime_error(message.str());
}
}  // namespace

YiyouMotor::YiyouMotor(iswv::canopen::CanopenMaster & master, std::uint8_t node_id)
: node_(master.node(node_id)), drive_(node_)
{
}

YiyouMotor::~YiyouMotor()
{
  try {
    stop();
  } catch (...) {
  }  // 显式 stop() 会把失败报告给调用者；析构只尽力停车。
}

double YiyouMotor::configure_rpm_units()
{
  if (active_) {throw std::logic_error("stop the motor before changing RPM scale");}
  pulses_per_output_revolution_ = 0;
  const auto encoder = node_->read(encoder_resolution);
  check(encoder, "read encoder resolution 0x2025");
  const auto numerator = node_->read(gear_numerator);
  check(numerator, "read reduction ratio numerator 0x26A2");
  const auto denominator = node_->read(gear_denominator);
  check(denominator, "read reduction ratio denominator 0x26A3");
  if (encoder.value() <= 0 || numerator.value() == 0 || denominator.value() == 0) {
    throw std::runtime_error("invalid encoder resolution/reduction ratio; cannot convert output RPM");
  }
  pulses_per_output_revolution_ = static_cast<double>(encoder.value()) *
    numerator.value() / denominator.value();
  return pulses_per_output_revolution_;
}

std::int32_t YiyouMotor::velocity_from_rpm(double rpm) const
{
  if (pulses_per_output_revolution_ <= 0) {
    throw std::logic_error("configure_rpm_units must succeed before using RPM");
  }
  if (!std::isfinite(rpm)) {throw std::invalid_argument("RPM must be finite");}
  const double raw = std::round(rpm * pulses_per_output_revolution_ / 60.0);
  if (!std::isfinite(raw) || raw < std::numeric_limits<std::int32_t>::min() ||
    raw > std::numeric_limits<std::int32_t>::max())
  {
    throw std::out_of_range("RPM conversion exceeds the signed 32-bit CANopen velocity range");
  }
  if (rpm != 0 && raw == 0) {
    throw std::out_of_range("RPM is below one pulse/s resolution");
  }
  return static_cast<std::int32_t>(raw);
}

void YiyouMotor::enable_rpm(double acceleration_rpm_per_s, double deceleration_rpm_per_s)
{
  if (acceleration_rpm_per_s <= 0 || deceleration_rpm_per_s <= 0) {
    throw std::invalid_argument("RPM/s acceleration and deceleration must be positive");
  }
  const auto acceleration = velocity_from_rpm(acceleration_rpm_per_s);
  const auto deceleration = velocity_from_rpm(deceleration_rpm_per_s);
  enable(static_cast<std::uint32_t>(acceleration), static_cast<std::uint32_t>(deceleration));
}

void YiyouMotor::set_velocity_rpm(double rpm)
{
  set_velocity(velocity_from_rpm(rpm));
}

double YiyouMotor::actual_velocity_rpm()
{
  if (pulses_per_output_revolution_ <= 0) {
    throw std::logic_error("configure_rpm_units must succeed before using RPM");
  }
  return static_cast<double>(actual_velocity()) * 60.0 / pulses_per_output_revolution_;
}

void YiyouMotor::enable(std::uint32_t acceleration, std::uint32_t deceleration)
{
  if (acceleration == 0 || deceleration == 0) {
    throw std::invalid_argument("acceleration and deceleration must be positive (pulse/s^2)");
  }
  if (active_) {throw std::logic_error("stop the motor before reconfiguring");}
  // 切到 CANopen 控制源后才允许写运动对象，避免串口/ EtherCAT 权限下误动作。
  const auto source = node_->read(control_source);
  check(source, "read control source 0x2100");
  if (source.value() != 2) {
    if (source.value() > 2) {
      throw std::runtime_error("unknown control source 0x2100; refusing to change authority");
    }
    const auto state = drive_.state();
    check(state, "check state before changing control source");
    if (state.value() != iswv::cia402::DriveState::switch_on_disabled &&
      state.value() != iswv::cia402::DriveState::ready_to_switch_on &&
      state.value() != iswv::cia402::DriveState::switched_on)
    {
      throw std::runtime_error(
              std::string("refusing to change control source in state: ") +
              iswv::cia402::state_name(state.value()));
    }
    check(node_->write(control_source, 2), "select CANopen control source 0x2100=2");
    const auto confirmed = node_->read(control_source);
    check(confirmed, "verify control source 0x2100");
    if (confirmed.value() != 2) {
      throw std::runtime_error("0x2100 did not confirm CANopen control; no motion command sent");
    }
  }
  active_ = true;  // 初始化中途失败也由 stop() 清理。
  try {
    check(node_->send_nmt(iswv::canopen::NmtCommand::enter_pre_operational), "NMT pre-op");
    check(drive_.write_control(0), "disable voltage");
    check(
      drive_.wait_for_status(
        [](std::uint16_t status) {
          return iswv::cia402::decode_state(status) ==
          iswv::cia402::DriveState::switch_on_disabled;
        }, 1000ms), "wait for disabled state (check fault/STO if timeout)");
    check(node_->write(target_velocity, 0), "clear 0x60FF");
    check(drive_.set_mode(iswv::cia402::OperationMode::profile_velocity), "set PV mode");
    const auto mode_deadline = std::chrono::steady_clock::now() + 1s;
    while (true) {
      const auto mode = node_->read(mode_display);
      check(mode, "read 0x6061 while waiting for PV mode");
      if (mode.value() == 3) {break;}
      if (std::chrono::steady_clock::now() >= mode_deadline) {
        throw std::runtime_error(
                "timeout waiting for 0x6061=3 (PV); last mode=" +
                std::to_string(static_cast<int>(mode.value())));
      }
      std::this_thread::sleep_for(10ms);
    }
    check(node_->write(profile_acceleration, acceleration), "write 0x6083");
    check(node_->write(profile_deceleration, deceleration), "write 0x6084");
    check(node_->send_nmt(iswv::canopen::NmtCommand::start), "NMT start");
    check(drive_.enable(), "CiA402 enable (0x06 -> 0x07 -> 0x0F)");
    enabled_ = true;
  } catch (...) {
    try {
      stop();
    } catch (...) {
    }
    throw;
  }
}

void YiyouMotor::set_velocity(std::int32_t pulses_per_second)
{
  if (!enabled_) {throw std::logic_error("motor is not enabled");}
  (void)actual_velocity();
  check(node_->write(target_velocity, pulses_per_second), "write 0x60FF");
}

std::int32_t YiyouMotor::actual_velocity()
{
  const auto status = drive_.read_status();
  check(status, "read 0x6041");
  if (iswv::cia402::decode_state(status.value()) !=
    iswv::cia402::DriveState::operation_enabled)
  {
    std::ostringstream message;
    message << "motor left operation-enabled state, 0x6041=0x" << std::hex << status.value();
    throw std::runtime_error(message.str());
  }
  const auto velocity = node_->read(velocity_actual);
  check(velocity, "read 0x606C");
  return velocity.value();
}

std::int32_t YiyouMotor::actual_position()
{
  const auto position = node_->read(Object<std::int32_t>{{0x6064, 0}, "actual position"});
  check(position, "read 0x6064");
  return position.value();
}

std::string YiyouMotor::diagnostic_report()
{
  std::ostringstream report;
  report << "DIAG node=" << static_cast<unsigned>(node_->id()) << '\n';
  const auto read_object = [this, &report](auto object) {
      report << "  0x" << std::hex << std::setfill('0') << std::setw(4)
             << object.address.index << ":00 " << object.name << " = ";
      const auto result = node_->read(object);
      if (result) {
        report << "0x" << std::setw(sizeof(result.value()) * 2)
               << static_cast<std::uint32_t>(result.value());
        if (object.address.index == 0x6041) {
          report << " (" << iswv::cia402::state_name(
            iswv::cia402::decode_state(static_cast<std::uint16_t>(result.value()))) << ")";
        }
        if (object.address.index == 0x2100) {
          const char * source_name = result.value() == 0 ? "UART" :
            (result.value() == 1 ? "EtherCAT" : (result.value() == 2 ? "CANopen" : "unknown"));
          report << " (" << source_name << ")";
        }
        report << '\n';
      } else {
        report << "READ FAILED: " << result.error().message;
        if (result.error().sdo_abort_code) {
          report << " (SDO abort 0x" << std::setw(8) << *result.error().sdo_abort_code << ")";
        }
        report << '\n';
      }
    };
  read_object(iswv::cia402::status_word);
  read_object(Object<std::uint16_t>{{0x603F, 0}, "error code"});
  read_object(Object<std::uint8_t>{{0x6061, 0}, "mode display"});
  read_object(control_source);
  return report.str();
}

void YiyouMotor::stop()
{
  if (!active_) {return;}
  std::exception_ptr failure;
  try {
    check(node_->write(target_velocity, 0), "stop: clear 0x60FF");
    if (enabled_) {
      const auto deadline = std::chrono::steady_clock::now() + 3s;
      while (true) {
        const auto speed = node_->read(velocity_actual);
        check(speed, "stop: read 0x606C");
        if (speed.value() >= -1 && speed.value() <= 1) {break;}
        if (std::chrono::steady_clock::now() >= deadline) {
          throw std::runtime_error("stop: zero-speed timeout; disabling voltage");
        }
        std::this_thread::sleep_for(20ms);
      }
    }
  } catch (...) {
    failure = std::current_exception();
  }
  // 即使零速命令或反馈读取失败，也要继续尝试下电。
  enabled_ = false;
  check(drive_.write_control(0), "stop: disable voltage");
  check(
    drive_.wait_for_status(
      [](std::uint16_t status) {
        return (status & iswv::cia402::status::operation_enabled) == 0;
      }, 1000ms), "stop: verify disabled");
  active_ = false;
  if (failure) {std::rethrow_exception(failure);}
}

}  // namespace chassis
