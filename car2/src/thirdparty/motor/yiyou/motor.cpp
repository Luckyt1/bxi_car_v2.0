#include "motor/yiyou/motor.h"

#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace chassis
{
namespace
{
using namespace std::chrono_literals;
using iswv::canopen::Object;

constexpr Object<std::int8_t> mode_display{{0x6061, 0}, "mode display"};
constexpr Object<std::int32_t> target_velocity{{0x60FF, 0}, "target velocity"};
constexpr Object<std::int32_t> velocity_actual{{0x606C, 0}, "actual velocity"};
constexpr Object<std::int32_t> target_position{{0x607A, 0}, "target position"};
constexpr Object<std::int16_t> target_current{{0x6071, 0}, "target current"};
constexpr Object<std::int16_t> current_actual{{0x6078, 0}, "actual current"};
constexpr Object<std::int16_t> torque_offset{{0x60B2, 0}, "torque offset"};
constexpr Object<std::uint32_t> profile_velocity{{0x6081, 0}, "profile velocity"};
constexpr Object<std::uint32_t> profile_acceleration{{0x6083, 0}, "acceleration"};
constexpr Object<std::uint32_t> profile_deceleration{{0x6084, 0}, "deceleration"};
constexpr Object<std::uint8_t> control_source{{0x2100, 0}, "control source"};
constexpr Object<std::int32_t> encoder_resolution{{0x2025, 0}, "encoder counts/revolution"};
constexpr Object<std::uint16_t> gear_numerator{{0x26A2, 0}, "gear numerator"};
constexpr Object<std::uint16_t> gear_denominator{{0x26A3, 0}, "gear denominator"};
constexpr Object<std::int8_t> motion_stop_flag{{0x2707, 0}, "motor stopped"};
constexpr Object<std::uint8_t> save_parameters{{0x2130, 0}, "save parameters to EEPROM"};
constexpr auto cst_mode = static_cast<iswv::cia402::OperationMode>(10);

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

YiyouMotor::YiyouMotor(ethercat::Master & master, std::uint16_t slave_position)
: YiyouMotor(master.node(slave_position))
{
}

YiyouMotor::YiyouMotor(std::shared_ptr<ethercat::Node> node)
: node_(std::move(node)), drive_(node_)
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
    throw std::out_of_range("RPM conversion exceeds the signed 32-bit CoE velocity range");
  }
  if (rpm != 0 && raw == 0) {
    throw std::out_of_range("RPM is below one pulse/s resolution");
  }
  return static_cast<std::int32_t>(raw);
}

std::int32_t YiyouMotor::position_from_degrees(double degrees) const
{
  if (pulses_per_output_revolution_ <= 0) {
    throw std::logic_error("configure_rpm_units must succeed before using degrees");
  }
  if (!std::isfinite(degrees)) {throw std::invalid_argument("degrees must be finite");}
  const double raw = std::round(degrees * pulses_per_output_revolution_ / 360.0);
  if (!std::isfinite(raw) || raw < std::numeric_limits<std::int32_t>::min() ||
    raw > std::numeric_limits<std::int32_t>::max())
  {
    throw std::out_of_range("angle exceeds the signed 32-bit CoE position range");
  }
  if (degrees != 0 && raw == 0) {
    throw std::out_of_range("angle is below one pulse resolution");
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
  enable_mode(iswv::cia402::OperationMode::profile_velocity, 0, acceleration, deceleration);
}

void YiyouMotor::enable_position(
  std::uint32_t velocity, std::uint32_t acceleration, std::uint32_t deceleration)
{
  if (velocity == 0) {throw std::invalid_argument("profile velocity must be positive (pulse/s)");}
  enable_mode(iswv::cia402::OperationMode::profile_position, velocity, acceleration, deceleration);
}

void YiyouMotor::disable_for_configuration()
{
  check(drive_.write_control(0), "disable voltage");
  check(
    drive_.wait_for_status(
      [](std::uint16_t status) {
        return iswv::cia402::decode_state(status) ==
        iswv::cia402::DriveState::switch_on_disabled;
      }, 1000ms), "wait for disabled state (check fault/STO if timeout)");
}

void YiyouMotor::wait_for_mode_display(const char * operation)
{
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (true) {
    const auto displayed = node_->read(mode_display);
    check(displayed, operation);
    if (displayed.value() == static_cast<std::int8_t>(mode_)) {return;}
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error(
              "timeout waiting for 0x6061=" +
              std::to_string(static_cast<int>(mode_)) + "; last mode=" +
              std::to_string(static_cast<int>(displayed.value())));
    }
    std::this_thread::sleep_for(10ms);
  }
}

void YiyouMotor::enable_cst()
{
  if (active_) {throw std::logic_error("stop the motor before reconfiguring");}
  position_target_.reset();
  select_ethercat_control();
  mode_ = cst_mode;
  active_ = true;  // 初始化中途失败也由 stop() 清理。
  try {
    disable_for_configuration();
    check(node_->write(target_current, static_cast<std::int16_t>(0)), "clear 0x6071");
    const auto confirmed_current = node_->read(target_current);
    check(confirmed_current, "verify target current 0x6071");
    if (confirmed_current.value() != 0) {
      throw std::runtime_error("0x6071 target current did not clear; no enable sent");
    }
    check(node_->write(torque_offset, static_cast<std::int16_t>(0)), "clear 0x60B2");
    const auto confirmed_offset = node_->read(torque_offset);
    check(confirmed_offset, "verify torque offset 0x60B2");
    if (confirmed_offset.value() != 0) {
      throw std::runtime_error("0x60B2 torque offset did not clear; no enable sent");
    }
    check(drive_.set_mode(mode_), "set CST operation mode 0x6060=10");
    wait_for_mode_display("read 0x6061 while waiting for CST mode");
    check(drive_.enable(), "CiA402 enable CST (0x06 -> 0x07 -> 0x0F)");
    enabled_ = true;
    check_current_mode();
  } catch (...) {
    try {
      stop();
    } catch (...) {
    }
    throw;
  }
}

void YiyouMotor::enable_mode(
  iswv::cia402::OperationMode mode, std::uint32_t velocity,
  std::uint32_t acceleration, std::uint32_t deceleration)
{
  if (acceleration == 0 || deceleration == 0) {
    throw std::invalid_argument("acceleration and deceleration must be positive (pulse/s^2)");
  }
  if (active_) {throw std::logic_error("stop the motor before reconfiguring");}
  position_target_.reset();
  select_ethercat_control();
  mode_ = mode;
  active_ = true;  // 初始化中途失败也由 stop() 清理。
  try {
    disable_for_configuration();
    if (mode_ == iswv::cia402::OperationMode::profile_position) {
      // 先覆盖历史目标；使能和 bit4 上升沿之前均不允许追逐旧位置。
      check(node_->write(target_position, actual_position()), "seed 0x607A with current position");
      check(node_->write(profile_velocity, velocity), "write 0x6081");
    } else {
      check(node_->write(target_velocity, 0), "clear 0x60FF");
    }
    check(drive_.set_mode(mode_), "set operation mode 0x6060");
    wait_for_mode_display("read 0x6061 while waiting for operation mode");
    check(node_->write(profile_acceleration, acceleration), "write 0x6083");
    check(node_->write(profile_deceleration, deceleration), "write 0x6084");
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

void YiyouMotor::select_ethercat_control()
{
  // 切到 EtherCAT 控制源后才允许写运动对象，避免串口/CANopen 权限下误动作。
  const auto source = node_->read(control_source);
  check(source, "read control source 0x2100");
  if (source.value() != 1) {
    if (source.value() > 2) {
      throw std::runtime_error("unknown control source 0x2100; refusing to change authority");
    }
    const auto state = drive_.state();
    check(state, "check state before changing control source");
    if (state.value() != iswv::cia402::DriveState::switch_on_disabled) {
      throw std::runtime_error(
              std::string("refusing to change control source in state: ") +
              iswv::cia402::state_name(state.value()));
    }
    check(node_->write(control_source, 1), "select EtherCAT control source 0x2100=1");
    const auto confirmed = node_->read(control_source);
    check(confirmed, "verify control source 0x2100");
    if (confirmed.value() != 1) {
      throw std::runtime_error("0x2100 did not confirm EtherCAT control; no motion command sent");
    }
  }
}

void YiyouMotor::save_zero_position(std::uint32_t tolerance_pulses)
{
  if (active_) {throw std::logic_error("stop the motor before setting zero");}
  const auto state = drive_.state();
  check(state, "check state before setting zero");
  if (state.value() != iswv::cia402::DriveState::switch_on_disabled &&
    state.value() != iswv::cia402::DriveState::ready_to_switch_on &&
    state.value() != iswv::cia402::DriveState::switched_on)
  {
    throw std::runtime_error("setting zero requires a disabled motor");
  }
  if (!motion_stopped()) {throw std::runtime_error("setting zero requires a stationary motor");}
  select_ethercat_control();
  position_target_.reset();
  mode_ = iswv::cia402::OperationMode::homing;
  active_ = true;
  try {
    check(drive_.write_control(0), "disable before setting zero");
    check(
      drive_.wait_for_status(
        [](std::uint16_t status) {
          return iswv::cia402::decode_state(status) ==
          iswv::cia402::DriveState::switch_on_disabled;
        }, 1000ms), "wait for disabled state before setting zero");
    // yy.pdf §4.7.4 的 HM 固定使用方式 35；实机写 0x6098 返回 abort 0x14。
    // 按厂家流程直接切 HM，再用 0x0010 触发；不访问 0x6098。
    check(drive_.set_mode(mode_), "select HM 0x6060=6");
    const auto mode_deadline = std::chrono::steady_clock::now() + 1s;
    while (true) {
      const auto displayed = node_->read(mode_display);
      check(displayed, "confirm HM 0x6061=6");
      if (displayed.value() == 6) {break;}
      if (std::chrono::steady_clock::now() >= mode_deadline) {
        throw std::runtime_error("timeout waiting for HM 0x6061=6");
      }
      std::this_thread::sleep_for(10ms);
    }
    // 厂家 yy.pdf §4.7.4：0x0010 原地设零；不能用通用 CiA402 的 0x001F。
    check(drive_.write_control(0x0010), "set current position as zero 0x6040=0x10");
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (true) {
      const auto status = drive_.read_status();
      check(status, "check state after setting zero");
      if (iswv::cia402::decode_state(status.value()) !=
        iswv::cia402::DriveState::switch_on_disabled)
      {
        throw std::runtime_error("motor must stay disabled while setting zero");
      }
      const auto position = static_cast<std::int64_t>(actual_position());
      const auto tolerance = static_cast<std::int64_t>(tolerance_pulses);
      if (position >= -tolerance && position <= tolerance) {break;}
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("zero position did not confirm; EEPROM save was not sent");
      }
      std::this_thread::sleep_for(10ms);
    }
    stop();  // 清除 HM 触发位后保存，防止后续切 PP 时沿用 bit4。
    check(node_->write(save_parameters, 1, {5s}), "save zero parameters 0x2130=1");
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
  if (mode_ != iswv::cia402::OperationMode::profile_velocity) {
    throw std::logic_error("velocity commands require PV mode");
  }
  (void)actual_velocity();
  check(node_->write(target_velocity, pulses_per_second), "write 0x60FF");
}

void YiyouMotor::check_current_mode()
{
  if (!enabled_ || mode_ != cst_mode) {
    throw std::logic_error("current commands require enabled CST mode");
  }
  const auto status = drive_.read_status();
  check(status, "read CST status 0x6041");
  if (iswv::cia402::decode_state(status.value()) !=
    iswv::cia402::DriveState::operation_enabled ||
    (status.value() & (iswv::cia402::status::fault |
    iswv::cia402::status::internal_limit)) != 0)
  {
    std::ostringstream message;
    message << "CST state/fault/internal limit, 0x6041=0x" << std::hex << status.value();
    throw std::runtime_error(message.str());
  }
  const auto displayed = node_->read(mode_display);
  check(displayed, "verify CST mode 0x6061");
  if (displayed.value() != 10) {throw std::runtime_error("motor left CST mode (0x6061 != 10)");}
}

void YiyouMotor::set_current_permille(std::int16_t permille)
{
  if (permille < -1000 || permille > 1000) {
    throw std::out_of_range("target current permille must be within [-1000, 1000]");
  }
  check_current_mode();
  check(node_->write(target_current, permille), "write 0x6071");
}

std::int16_t YiyouMotor::actual_current_permille()
{
  const auto current = node_->read(current_actual);
  check(current, "read 0x6078");
  return current.value();
}

std::uint16_t YiyouMotor::position_status()
{
  if (!enabled_ || mode_ != iswv::cia402::OperationMode::profile_position) {
    throw std::logic_error("position commands require enabled PP mode");
  }
  const auto status = drive_.read_status();
  check(status, "read PP status 0x6041");
  // 按手册通用状态字表检查 bit13 跟随误差（PP 专章把该位列为保留位）。
  if (iswv::cia402::decode_state(status.value()) !=
    iswv::cia402::DriveState::operation_enabled ||
    (status.value() & (iswv::cia402::status::following_error |
    iswv::cia402::status::internal_limit)) != 0)
  {
    std::ostringstream message;
    message << "PP state/following error/internal limit, 0x6041=0x" << std::hex << status.value();
    throw std::runtime_error(message.str());
  }
  const auto displayed = node_->read(mode_display);
  check(displayed, "verify PP mode 0x6061");
  if (displayed.value() != 1) {throw std::runtime_error("motor left PP mode (0x6061 != 1)");}
  return status.value();
}

void YiyouMotor::wait_for_position_ack(bool acknowledged)
{
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (true) {
    const bool ack = (position_status() & iswv::cia402::status::mode_specific_12) != 0;
    if (ack == acknowledged) {return;}
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error("timeout waiting for PP set-point acknowledge (0x6041 bit12)");
    }
    std::this_thread::sleep_for(10ms);
  }
}

void YiyouMotor::move_to_position(std::int32_t position)
{
  if (position_command_ != PositionCommand::idle) {
    throw std::logic_error("PP command is still pending");
  }
  if (!enabled_ || mode_ != iswv::cia402::OperationMode::profile_position) {
    throw std::logic_error("position commands require enabled PP mode");
  }
  position_target_.reset();
  try {
    (void)position_status();
    check(drive_.write_control(iswv::cia402::control::absolute_prepare), "PP clear new set-point");
    wait_for_position_ack(false);
    check(node_->write(target_position, position), "write 0x607A");
    const auto confirmed = node_->read(target_position);
    check(confirmed, "verify target position 0x607A");
    if (confirmed.value() != position) {
      throw std::runtime_error("0x607A target position readback mismatch; no set-point trigger sent");
    }
    check(
      drive_.write_control(
        iswv::cia402::control::absolute_start), "PP new set-point rising edge");
    wait_for_position_ack(true);
    check(drive_.write_control(iswv::cia402::control::absolute_prepare), "PP reset new set-point");
    wait_for_position_ack(false);
    position_target_ = position;
  } catch (...) {
    try {
      stop();
    } catch (...) {
    }
    throw;
  }
}

void YiyouMotor::prepare_position_command(std::int32_t position)
{
  if (position_command_ != PositionCommand::idle) {
    throw std::logic_error("PP command is still pending");
  }
  (void)position_status();
  check(drive_.write_control(iswv::cia402::control::absolute_prepare), "PP clear new set-point");
  pending_position_ = position;
  position_command_ = PositionCommand::clearing;
}

bool YiyouMotor::stage_position_command()
{
  if (position_command_ != PositionCommand::clearing) {
    throw std::logic_error("PP command was not prepared");
  }
  if ((position_status() & iswv::cia402::status::mode_specific_12) != 0) {return false;}
  check(node_->write(target_position, pending_position_, {5ms}), "stage 0x607A");
  const auto confirmed = node_->read(target_position, {5ms});
  check(confirmed, "verify staged 0x607A");
  if (confirmed.value() != pending_position_) {
    throw std::runtime_error("0x607A staged target mismatch; no set-point trigger sent");
  }
  position_command_ = PositionCommand::staged;
  return true;
}

void YiyouMotor::trigger_position_command()
{
  if (position_command_ != PositionCommand::staged) {
    throw std::logic_error("PP target was not staged");
  }
  (void)position_status();
  check(drive_.write_control(iswv::cia402::control::absolute_start), "PP trigger staged target");
  position_target_.reset();
  position_command_ = PositionCommand::triggered;
}

bool YiyouMotor::finish_position_command()
{
  if (position_command_ == PositionCommand::idle) {return true;}
  const bool ack = (position_status() & iswv::cia402::status::mode_specific_12) != 0;
  if (position_command_ == PositionCommand::triggered) {
    if (ack) {
      check(
        drive_.write_control(iswv::cia402::control::absolute_prepare),
        "PP reset new set-point");
      position_command_ = PositionCommand::resetting;
    }
    return false;
  }
  if (position_command_ != PositionCommand::resetting) {
    throw std::logic_error("PP target was not triggered");
  }
  if (ack) {return false;}
  position_target_ = pending_position_;
  position_command_ = PositionCommand::idle;
  return true;
}

bool YiyouMotor::position_reached(std::uint32_t tolerance_pulses)
{
  if (!enabled_ || mode_ != iswv::cia402::OperationMode::profile_position) {
    throw std::logic_error("position commands require enabled PP mode");
  }
  try {
    const auto status = position_status();
    if (!position_target_ || (status & iswv::cia402::status::target_reached) == 0) {
      return false;
    }
    // 切换模式也会置位 bit10；只接受本次目标的实际位置，短行程无需观察 bit10 下降沿。
    const auto error = static_cast<std::int64_t>(actual_position()) - *position_target_;
    const auto tolerance = static_cast<std::int64_t>(tolerance_pulses);
    return error >= -tolerance && error <= tolerance;
  } catch (...) {
    try {
      stop();
    } catch (...) {
    }
    throw;
  }
}

bool YiyouMotor::motion_stopped()
{
  const auto stopped = node_->read(motion_stop_flag);
  check(stopped, "read 0x2707");
  if (stopped.value() == 0) {return false;}
  if (stopped.value() == 1) {return true;}
  throw std::runtime_error("0x2707 motor stopped flag must be 0 or 1");
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
  report << "DIAG ethercat_slave=" << node_->id() << '\n';
  const auto read_object = [this, &report](auto object) {
      report << "  0x" << std::hex << std::setfill('0') << std::setw(4)
             << object.address.index << ':' << std::setw(2)
             << static_cast<unsigned>(object.address.subindex) << ' ' << object.name << " = ";
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
  read_object(Object<std::uint32_t>{{0x1018, 1}, "vendor ID"});
  read_object(Object<std::uint32_t>{{0x1018, 2}, "product code"});
  read_object(Object<std::uint32_t>{{0x1018, 3}, "revision"});
  read_object(Object<std::uint32_t>{{0x1018, 4}, "serial number"});
  read_object(encoder_resolution);
  read_object(gear_numerator);
  read_object(gear_denominator);
  read_object(Object<std::int32_t>{{0x6064, 0}, "actual position"});
  read_object(velocity_actual);
  read_object(motion_stop_flag);
  return report.str();
}

void YiyouMotor::stop()
{
  position_command_ = PositionCommand::idle;
  position_target_.reset();
  if (!active_) {return;}
  std::exception_ptr failure;
  try {
    if (mode_ == iswv::cia402::OperationMode::profile_position) {
      if (enabled_) {
        // PP 下 0x60FF 无效；置 halt (bit8)，再等待减速、失能。
        check(drive_.write_control(0x010F), "stop: PP halt");
      }
    } else if (mode_ == iswv::cia402::OperationMode::profile_velocity) {
      check(node_->write(target_velocity, 0), "stop: clear 0x60FF");
    } else if (mode_ == cst_mode) {
      check(node_->write(target_current, static_cast<std::int16_t>(0)), "stop: clear 0x6071");
    }
    if (enabled_ && mode_ != cst_mode) {
      const auto deadline = std::chrono::steady_clock::now() + 3s;
      while (true) {
        if (motion_stopped()) {break;}
        if (std::chrono::steady_clock::now() >= deadline) {
          throw std::runtime_error("stop: motor-stopped timeout; disabling voltage");
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
