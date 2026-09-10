#include "chassis/iswv_drive.hpp"

#include "iswv/cia402/drive.hpp"
#include "iswv/iswv/object_dictionary.hpp"

#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <mutex>

namespace chassis
{
namespace
{

using namespace std::chrono_literals;

iswv::canopen::SdoOptions sdo_options()
{
  return {250ms};
}

iswv::cia402::TransitionOptions transition_options()
{
  return {1000ms, 10ms, sdo_options()};
}

void throw_error(const char * action, const iswv::Error & error)
{
  std::ostringstream message;
  message << "iSWV drive " << action << " failed: " << error.message;
  if (error.sdo_abort_code) {
    message << " (SDO abort 0x" << std::hex << *error.sdo_abort_code << ')';
  }
  throw std::runtime_error(message.str());
}

template<typename T>
T require_result(iswv::Result<T> result, const char * action)
{
  if (!result) {
    throw_error(action, result.error());
  }
  return result.take_value();
}

void require_result(iswv::Result<void> result, const char * action)
{
  if (!result) {
    throw_error(action, result.error());
  }
}

double output_rpm_to_motor_rpm(double output_rpm, const iswv::AxisConfiguration & config)
{
  if (!std::isfinite(output_rpm)) {
    throw std::invalid_argument("iSWV drive output RPM must be finite");
  }
  return output_rpm * config.gear_ratio;
}

double output_rpm_s_to_motor_rps2(
  double output_rpm_s,
  const iswv::AxisConfiguration & config)
{
  if (!std::isfinite(output_rpm_s) || output_rpm_s < 0.0) {
    throw std::invalid_argument(
            "iSWV drive acceleration/deceleration must be finite and non-negative");
  }
  return output_rpm_s * config.gear_ratio / 60.0;
}

iswv::AxisConfiguration validated_travel_config(iswv::AxisConfiguration config)
{
  if (config.encoder_resolution == 0 || !std::isfinite(config.gear_ratio) ||
    config.gear_ratio <= 0.0)
  {
    throw std::invalid_argument(
            "iSWV drive requires positive gear ratio and non-zero encoder resolution");
  }
  config.kind = iswv::AxisKind::travel;
  config.inverted = false;
  return config;
}

}  // namespace

class IswvDrive::Impl
{
public:
  struct Feedback
  {
    std::mutex mutex;
    std::array<iswv::canopen::PdoEvent, 3> frames{};
  };
  Impl(
    iswv::canopen::CanopenMaster & master_in,
    iswv::AxisConfiguration config_in)
  : master(master_in),
    config(validated_travel_config(std::move(config_in))),
    axis(master.node(config.node_id), config)
  {
    for (std::uint8_t number = 1; number <= 3; ++number) {
      subscriptions[number - 1] = axis.node()->on_tpdo(
        number,
        [cache = feedback, number](const iswv::canopen::PdoEvent & event) {
          const auto expected_size = number == 3 ? 8U : 6U;
          if (event.size != expected_size) {return;}
          std::lock_guard<std::mutex> lock(cache->mutex);
          cache->frames[number - 1] = event;
        });
    }
  }

  double actual_velocity_pdo_rpm()
  {
    std::lock_guard<std::mutex> lock(feedback->mutex);
    const auto now = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < feedback->frames.size(); ++i) {
      const auto & frame = feedback->frames[i];
      if (frame.received_at == std::chrono::steady_clock::time_point{} ||
        now - frame.received_at > 250ms)
      {
        throw std::runtime_error(
                "iSWV Node-ID " + std::to_string(config.node_id) +
                " TPDO" + std::to_string(i + 1) + " feedback missing or older than 250 ms");
      }
    }
    const auto & status_frame = feedback->frames[0];
    const auto status = iswv::canopen::decode_little_endian<std::uint16_t>(
      status_frame.data.data(), 2).value();
    const auto & errors = feedback->frames[2];
    const auto error1 = iswv::canopen::decode_little_endian<std::uint16_t>(
      errors.data.data() + 4, 2).value();
    const auto error2 = iswv::canopen::decode_little_endian<std::uint16_t>(
      errors.data.data() + 6, 2).value();
    if (iswv::cia402::decode_state(status) != iswv::cia402::DriveState::operation_enabled ||
      error1 != 0 || error2 != 0)
    {
      std::ostringstream message;
      message << "iSWV Node-ID " << static_cast<unsigned>(config.node_id)
              << " PDO fault: status=0x" << std::hex << status
              << " error1=0x" << error1 << " error2=0x" << error2;
      throw std::runtime_error(message.str());
    }
    const auto raw = iswv::canopen::decode_little_endian<std::int32_t>(
      feedback->frames[1].data.data(), 4).value();
    return axis.raw_to_motor_rpm(raw) / config.gear_ratio;
  }

  void enable_rpm(
    double acceleration_output_rpm_s,
    double deceleration_output_rpm_s)
  {
    const auto acceleration =
      output_rpm_s_to_motor_rps2(acceleration_output_rpm_s, config);
    const auto deceleration =
      output_rpm_s_to_motor_rps2(deceleration_output_rpm_s, config);

    require_result(
      master.send_nmt(
        iswv::canopen::NmtCommand::enter_pre_operational,
        config.node_id),
      "enter pre-operational");
    require_result(
      axis.drive().write_control(
        iswv::cia402::control::disable_voltage,
        sdo_options()),
      "disable voltage");
    require_result(
      axis.command_motor_velocity_rpm(0.0, sdo_options()),
      "clear target velocity");
    require_result(
      axis.configure_default_pdos({}, sdo_options()),
      "configure PDOs");
    require_result(
      axis.configure_velocity_mode(
        false, acceleration, deceleration,
        sdo_options()),
      "configure profile velocity mode");
    const auto mode = require_result(
      axis.node()->read(
        iswv::objects::mode,
        sdo_options()),
      "read operation mode");
    if (mode != static_cast<std::int8_t>(
        iswv::cia402::OperationMode::profile_velocity))
    {
      throw std::runtime_error("iSWV drive mode readback is not profile velocity");
    }

    iswv::SafetyPolicy policy;
    policy.enabled = true;
    policy.configure_heartbeat_producer = true;
    policy.configure_device_interruption_fault = true;
    policy.heartbeat_producer_time = 100ms;
    policy.local_heartbeat_timeout = 350ms;
    policy.timeout_action = iswv::TimeoutAction::quick_stop;
    require_result(
      axis.apply_safety_policy(policy, sdo_options()),
      "configure heartbeat policy");

    require_result(
      master.send_nmt(iswv::canopen::NmtCommand::start, config.node_id),
      "start node");
    require_result(axis.drive().enable(transition_options()), "enable");
    enabled = true;
  }

  void set_velocity_rpm(double output_rpm)
  {
    require_result(
      axis.command_motor_velocity_rpm(
        output_rpm_to_motor_rpm(output_rpm, config), sdo_options()),
      "set target velocity");
  }

  double actual_velocity_rpm()
  {
    const auto status = require_result(
      axis.drive().read_status(sdo_options()),
      "read status");
    const auto drive_state = iswv::cia402::decode_state(status);
    if (drive_state == iswv::cia402::DriveState::fault ||
      drive_state == iswv::cia402::DriveState::fault_reaction_active)
    {
      throw std::runtime_error("iSWV drive is in fault state");
    }
    if (enabled && drive_state != iswv::cia402::DriveState::operation_enabled) {
      throw std::runtime_error("iSWV drive is not operation enabled");
    }
    const auto error_status = require_result(
      axis.node()->read(
        iswv::objects::error_status,
        sdo_options()),
      "read error status");
    const auto error_status_2 = require_result(
      axis.node()->read(iswv::objects::error_status_2, sdo_options()),
      "read error status 2");
    if (error_status != 0 || error_status_2 != 0) {
      std::ostringstream message;
      message << "iSWV drive fault status is non-zero: error_status=0x"
              << std::hex << error_status << ", error_status_2=0x"
              << error_status_2;
      throw std::runtime_error(message.str());
    }
    const auto velocity_raw = require_result(
      axis.node()->read(iswv::objects::actual_velocity, sdo_options()),
      "read actual velocity");
    return axis.raw_to_motor_rpm(velocity_raw) / config.gear_ratio;
  }

  void stop()
  {
    iswv::Error first_error;
    bool failed = false;
    auto remember = [&](iswv::Result<void> result) {
        if (!result && !failed) {
          first_error = result.error();
          failed = true;
        }
      };

    remember(axis.command_motor_velocity_rpm(0.0, sdo_options()));
    remember(axis.drive().quick_stop(sdo_options()));
    remember(
      axis.drive().write_control(
        iswv::cia402::control::disable_voltage,
        sdo_options()));
    enabled = false;

    if (failed) {
      throw_error("stop", first_error);
    }
  }

  iswv::canopen::CanopenMaster & master;
  iswv::AxisConfiguration config;
  iswv::Axis axis;
  bool enabled{false};
  std::shared_ptr<Feedback> feedback = std::make_shared<Feedback>();
  std::array<iswv::Subscription, 3> subscriptions;
};

IswvDrive::IswvDrive(
  iswv::canopen::CanopenMaster & master,
  iswv::AxisConfiguration config)
: impl_(std::make_unique<Impl>(master, std::move(config)))
{
}

IswvDrive::~IswvDrive()
{
  if (!impl_) {
    return;
  }
  try {
    impl_->stop();
  } catch (...) {
  }
}

double IswvDrive::actual_velocity_pdo_rpm()
{
  return impl_->actual_velocity_pdo_rpm();
}

void IswvDrive::set_velocity_pdo_rpm(double output_rpm)
{
  require_result(
    impl_->axis.command_velocity_pdo(
      output_rpm_to_motor_rpm(output_rpm, impl_->config)), "set velocity PDO");
}

void IswvDrive::enable_rpm(
  double acceleration_output_rpm_s,
  double deceleration_output_rpm_s)
{
  try {
    impl_->enable_rpm(acceleration_output_rpm_s, deceleration_output_rpm_s);
  } catch (...) {
    try {
      impl_->stop();
    } catch (...) {
    }
    throw;
  }
}

void IswvDrive::set_velocity_rpm(double output_rpm)
{
  try {
    impl_->set_velocity_rpm(output_rpm);
  } catch (...) {
    try {
      impl_->stop();
    } catch (...) {
    }
    throw;
  }
}

double IswvDrive::actual_velocity_rpm()
{
  return impl_->actual_velocity_rpm();
}

void IswvDrive::stop()
{
  impl_->stop();
}

}  // namespace chassis
