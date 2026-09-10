#include "chassis/eyou_motor.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip>
#include <limits>
#include <optional>
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

EyouMotor::EyouMotor(iswv::canopen::CanopenMaster & master, std::uint8_t node_id)
: node_(master.node(node_id)), drive_(node_)
{
}

EyouMotor::~EyouMotor()
{
  try {
    stop();
  } catch (...) {
  }                             // Explicit stop() reports failures to the caller.
}

double EyouMotor::configure_rpm_units()
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

std::int32_t EyouMotor::velocity_from_rpm(double rpm) const
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

void EyouMotor::enable_rpm(double acceleration_rpm_per_s, double deceleration_rpm_per_s)
{
  if (acceleration_rpm_per_s <= 0 || deceleration_rpm_per_s <= 0) {
    throw std::invalid_argument("RPM/s acceleration and deceleration must be positive");
  }
  const auto acceleration = velocity_from_rpm(acceleration_rpm_per_s);
  const auto deceleration = velocity_from_rpm(deceleration_rpm_per_s);
  enable(static_cast<std::uint32_t>(acceleration), static_cast<std::uint32_t>(deceleration));
}

void EyouMotor::set_velocity_rpm(double rpm)
{
  set_velocity(velocity_from_rpm(rpm));
}

double EyouMotor::actual_velocity_rpm()
{
  if (pulses_per_output_revolution_ <= 0) {
    throw std::logic_error("configure_rpm_units must succeed before using RPM");
  }
  return static_cast<double>(actual_velocity()) * 60.0 / pulses_per_output_revolution_;
}

void EyouMotor::enable(std::uint32_t acceleration, std::uint32_t deceleration)
{
  if (acceleration == 0 || deceleration == 0) {
    throw std::invalid_argument("acceleration and deceleration must be positive (pulse/s^2)");
  }
  if (active_) {throw std::logic_error("stop the motor before reconfiguring");}
  // yy.pdf p.183: 0=UART, 1=EtherCAT (factory default), 2=CANopen.
  // Obtain and verify CANopen authority before attempting motion-object writes.
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
  active_ = true;  // Also clean up partially completed initialization.
  try {
    check(node_->send_nmt(iswv::canopen::NmtCommand::enter_pre_operational), "NMT pre-op");
    // Remove any previous enable and stale velocity before changing mode.
    check(drive_.write_control(0), "disable voltage");
    check(
      drive_.wait_for_status(
        [](std::uint16_t status) {
          return iswv::cia402::decode_state(status) ==
          iswv::cia402::DriveState::switch_on_disabled;
        }, 1000ms), "wait for disabled state (check fault/STO if timeout)");
    check(node_->write(target_velocity, 0), "clear 0x60FF");
    check(drive_.set_mode(iswv::cia402::OperationMode::profile_velocity), "set PV mode");
    // An SDO write acknowledgement can precede the firmware's mode change.
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

void EyouMotor::set_velocity(std::int32_t pulses_per_second)
{
  if (!enabled_) {throw std::logic_error("motor is not enabled");}
  (void)actual_velocity();
  check(node_->write(target_velocity, pulses_per_second), "write 0x60FF");
}

std::int32_t EyouMotor::actual_velocity()
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

std::int32_t EyouMotor::actual_position()
{
  const auto position = node_->read(Object<std::int32_t>{{0x6064, 0}, "actual position"});
  check(position, "read 0x6064");
  return position.value();
}

std::string EyouMotor::diagnostic_report()
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

std::string EyouMotor::information_report(bool * complete)
{
  bool all_ok = true;
  bool scale_ok = false;
  std::int32_t encoder = 0;
  std::uint16_t numerator = 0;
  std::uint16_t denominator = 0;
  double pulses_per_output_revolution = 0;
  std::ostringstream report;
  report << "INFO node=" << static_cast<unsigned>(node_->id()) << '\n';

  const auto append_error = [&report](const auto & result) {
      report << "READ FAILED: " << result.error().message;
      if (result.error().sdo_abort_code) {
        report << " (SDO abort 0x" << std::hex << std::setfill('0') << std::setw(8)
               << *result.error().sdo_abort_code << std::dec << ")";
      }
      report << '\n';
    };

  const auto read_object = [this, &report, &append_error, &all_ok](auto object, auto print_value) {
      report << "  0x" << std::hex << std::setfill('0') << std::setw(4)
             << object.address.index << ":00 " << object.name << " = " << std::dec;
      const auto result = node_->read(object);
      if (!result) {
        all_ok = false;
        append_error(result);
        return false;
      }
      print_value(result.value());
      report << '\n' << std::dec;
      return true;
    };

  read_object(
    iswv::cia402::status_word,
    [&report](std::uint16_t value) {
      report << "0x" << std::hex << std::setfill('0') << std::setw(4) << value
             << " (" << iswv::cia402::state_name(iswv::cia402::decode_state(value)) << ")";
    });
  read_object(
    Object<std::uint16_t>{{0x603F, 0}, "error code"},
    [&report](std::uint16_t value) {
      report << "0x" << std::hex << std::setfill('0') << std::setw(4) << value;
    });
  read_object(mode_display, [&report](std::int8_t value) {report << static_cast<int>(value);});
  read_object(
    control_source,
    [&report](std::uint8_t value) {
      const char * source_name = value == 0 ? "UART" :
      (value == 1 ? "EtherCAT" : (value == 2 ? "CANopen" : "unknown"));
      report << static_cast<unsigned>(value) << " (" << source_name << ")";
    });

  const bool encoder_ok = read_object(
    encoder_resolution,
    [&report, &encoder](std::int32_t value) {encoder = value; report << value;});
  const bool numerator_ok = read_object(
    gear_numerator,
    [&report, &numerator](std::uint16_t value) {numerator = value; report << value;});
  const bool denominator_ok = read_object(
    gear_denominator,
    [&report, &denominator](std::uint16_t value) {denominator = value; report << value;});

  if (encoder_ok && numerator_ok && denominator_ok && encoder > 0 && numerator != 0 &&
    denominator != 0)
  {
    pulses_per_output_revolution = static_cast<double>(encoder) * numerator / denominator;
    scale_ok = true;
    report << "  output scale = " << std::fixed << std::setprecision(3)
           << pulses_per_output_revolution << " pulse/revolution\n";
  } else {
    all_ok = false;
    report << "  output RPM unavailable: invalid encoder resolution/reduction ratio\n";
  }

  const auto print_speed = [&report, scale_ok, pulses_per_output_revolution](std::int32_t value) {
      report << value << " pulse/s";
      if (scale_ok) {
        report << " (" << std::fixed << std::setprecision(3)
               << static_cast<double>(value) * 60.0 / pulses_per_output_revolution
               << " output RPM)";
      }
    };
  read_object(target_velocity, print_speed);
  read_object(velocity_actual, print_speed);

  if (complete) {*complete = all_ok;}
  return report.str();
}

std::string EyouMotor::electrical_report(bool * complete)
{
  const auto read = [this](auto object, std::string & error) -> std::optional<double> {
      const auto result = node_->read(object, iswv::canopen::SdoOptions{50ms});
      if (result) {return static_cast<double>(result.value());}
      std::ostringstream reason;
      reason << "0x" << std::hex << object.address.index << ": " << result.error().message;
      if (result.error().sdo_abort_code) {
        reason << " (SDO abort 0x" << std::setfill('0') << std::setw(8)
               << *result.error().sdo_abort_code << ")";
      }
      if (!error.empty()) {error += "; ";}
      error += reason.str();
      return std::nullopt;
    };

  std::string voltage_error, current_error, torque_error, temperature_error, power_error,
    force_error;
  const auto voltage = read(Object<std::uint32_t>{{0x6079, 0}, "bus voltage"}, voltage_error);
  const auto rated_current = read(
    Object<std::uint32_t>{{0x6075, 0}, "rated current"}, current_error);
  const auto current = read(Object<std::int16_t>{{0x6078, 0}, "actual current"}, current_error);
  const auto rated_torque = read(Object<std::uint32_t>{{0x6076, 0}, "rated torque"}, torque_error);
  const auto torque = read(Object<std::int16_t>{{0x6077, 0}, "actual torque"}, torque_error);
  const auto temperature = read(
    Object<std::int32_t>{{0x2779, 0}, "internal temperature"}, temperature_error);
  const auto power_temperature = read(
    Object<std::int32_t>{{0x277A, 0}, "power temperature"}, power_error);
  const auto force_torque = read(Object<std::int32_t>{{0x27BD, 0}, "force torque"}, force_error);
  if (rated_current && *rated_current == 0) {current_error += "额定电流为零";}
  if (rated_torque && *rated_torque == 0) {torque_error += "额定力矩为零";}

  bool all_ok = true;
  std::ostringstream report;
  report << std::fixed << std::setprecision(3);
  const auto append = [&report, &all_ok](
    const char * label, double value, const char * unit, const std::string & error) {
      report << label << ": ";
      if (error.empty()) {
        report << value << ' ' << unit;
      } else {
        all_ok = false;
        report << "不可用(" << error << ")";
      }
    };
  append("电压", voltage.value_or(0) / 1000.0, "V", voltage_error);
  report << "  ";
  append("电流", current.value_or(0) * rated_current.value_or(0) / 1e6, "A", current_error);
  report << "  ";
  append("电机力矩", torque.value_or(0) * rated_torque.value_or(0) / 1e6, "Nm", torque_error);
  report << "  ";
  append("内部温度", temperature.value_or(0), "C", temperature_error);
  report << "  ";
  append("功率温度", power_temperature.value_or(0), "C", power_error);
  report << "  ";
  append("力控力矩", force_torque.value_or(0), "Nm", force_error);
  if (complete) {*complete = all_ok;}
  return report.str();
}

void EyouMotor::stop()
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
  // Always attempt disable, even when the zero-speed command/read failed.
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
