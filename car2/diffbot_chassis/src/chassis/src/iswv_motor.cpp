#include "chassis/motor/iswv_motor.hpp"
#include "chassis/steering_tuning.hpp"
#include "chassis/torque_limit_detector.hpp"

#include "iswv/cia402/drive.hpp"
#include "iswv/iswv/object_dictionary.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

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

iswv::AxisConfiguration steering_axis_configuration(
  const SteeringConfigFile & config, std::size_t index)
{
  if (index >= 4) {
    throw std::out_of_range("invalid steering module index");
  }
  const auto modules = resolved_modules(config);
  const auto & module = modules[index];
  auto axis_config = iswv::AxisConfiguration::manual_steering(config.motor.node_ids[index]);
  axis_config.encoder_resolution = module.steering_encoder_resolution;
  axis_config.gear_ratio = module.steering_gear_ratio;
  axis_config.wheel_diameter_m = module.wheel_radius_m * 2.0;
  axis_config.inverted = module.steering_inverted;
  return axis_config;
}

SteeringAxes::SteeringAxes(
  iswv::canopen::CanopenMaster & can0, const SteeringConfigFile & config)
{
  validate_chassis_config(config);
  for (std::size_t index = 0; index < axes_.size(); ++index) {
    const auto axis_config = steering_axis_configuration(config, index);
    axes_[index] = std::make_shared<iswv::Axis>(can0.node(axis_config.node_id), axis_config);
  }
}

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

  IswvDrive::Feedback actual_feedback_pdo()
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
    return {
      axis.raw_to_motor_rpm(raw) / config.gear_ratio,
      iswv::canopen::decode_little_endian<std::int32_t>(status_frame.data.data() + 2, 4).value(),
      iswv::canopen::decode_little_endian<std::int16_t>(
        feedback->frames[1].data.data() + 4, 2).value()};
  }

  void enable_rpm(
    double acceleration_output_rpm_s,
    double deceleration_output_rpm_s,
    SteeringConfigFile * tuning_config = nullptr, std::size_t wheel_index = 0)
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
    if (tuning_config) {
      require_result(
        axis.drive().transition_to(
          iswv::cia402::DriveState::switch_on_disabled,
          transition_options()),
        "wait for disabled state before travel tuning");
      apply_drive_tuning(axis, *tuning_config, wheel_index);
    }
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
    auto enable_result = axis.drive().enable(transition_options());
    if (!enable_result && enable_result.error().code == iswv::ErrorCode::timeout) {
      const auto first_error = enable_result.error();
      // A timed-out controlword write may already have changed the drive state.
      // Read back zero and fault registers before resuming the state machine;
      // never reset faults or blindly replay the entire configuration sequence.
      const auto target = require_result(
        axis.node()->read(iswv::objects::target_velocity, sdo_options()),
        "verify zero target after enable timeout");
      if (target != 0) {
        throw std::runtime_error("iSWV enable retry refused: target velocity is not zero");
      }
      const auto error1 = require_result(
        axis.node()->read(iswv::objects::error_status, sdo_options()),
        "verify error status after enable timeout");
      const auto error2 = require_result(
        axis.node()->read(iswv::objects::error_status_2, sdo_options()),
        "verify error status 2 after enable timeout");
      if (error1 != 0 || error2 != 0) {
        std::ostringstream message;
        message << "iSWV enable retry refused: error1=0x" << std::hex << error1
                << " error2=0x" << error2;
        throw std::runtime_error(message.str());
      }
      std::cerr << "iSWV Node-ID " << static_cast<unsigned>(config.node_id)
                << " enable timeout: " << first_error.message
                << "; zero target and clear fault registers confirmed; retrying once\n";
      enable_result = axis.drive().enable(transition_options());
      if (!enable_result) {
        const auto action = "enable retry after " + first_error.message;
        throw_error(action.c_str(), enable_result.error());
      }
    }
    require_result(std::move(enable_result), "enable");
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
  return actual_feedback_pdo().output_rpm;
}

IswvDrive::Feedback IswvDrive::actual_feedback_pdo()
{
  return impl_->actual_feedback_pdo();
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

void IswvDrive::enable_rpm(
  double acceleration_output_rpm_s,
  double deceleration_output_rpm_s,
  SteeringConfigFile & tuning_config, std::size_t wheel_index)
{
  try {
    impl_->enable_rpm(
      acceleration_output_rpm_s, deceleration_output_rpm_s, &tuning_config, wheel_index);
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


namespace
{

using namespace std::chrono_literals;

constexpr double kPi = 3.14159265358979323846;
constexpr std::array<const char *, 4> kWheelNames{
  "front_left", "rear_left", "rear_right", "front_right"};
constexpr std::array<std::size_t, 4> kAllAxes{{0, 1, 2, 3}};

double position_to_rad(
  const SwerveModuleConfig & module, std::int32_t position,
  std::int32_t zero_offset)
{
  const auto relative_inc = static_cast<std::int64_t>(position) - zero_offset;
  return static_cast<double>(module.steering_sign) * static_cast<double>(relative_inc) *
         2.0 * kPi /
         (static_cast<double>(module.steering_encoder_resolution) * module.steering_gear_ratio);
}

double output_torque_nm(
  const SteeringConfigFile & config, const SwerveModuleConfig & module,
  std::int16_t current_raw)
{
  const double current_a = static_cast<double>(current_raw) *
    (config.motor_peak_current_a / 1.414) / 2048.0;
  return std::abs(current_a) * config.motor_torque_constant_nm_per_a *
         module.steering_gear_ratio;
}

void require_running(const std::function<bool()> & keep_running)
{
  if (keep_running && !keep_running()) {
    throw std::runtime_error("steering calibration cancelled");
  }
}

template<typename T>
void require_success(const iswv::Result<T> & result, const std::string & message)
{
  if (!result) {
    throw std::runtime_error(message + ": " + result.error().message);
  }
}

class SteeringCalibrator final
{
public:
  template<typename Steering>
  SteeringCalibrator(
    Steering & steering, SteeringConfigFile & config,
    const std::function<bool()> & keep_running)
  : config_(config), keep_running_(keep_running), modules_(resolved_modules(config))
  {
    validate_chassis_config(config_);
    for (std::size_t index = 0; index < axes_.size(); ++index) {
      axes_[index] = steering.at(static_cast<iswv::WheelPosition>(index));
      search_motor_rpm_[index] = calibration_motor_search_rpm(config_, index);
    }
  }

  ~SteeringCalibrator()
  {
    if (!success_) {
      stop_all();
    }
  }

  SteeringCalibrationResult run()
  {
    std::cout << "将用输出轴速度 " << config_.calibration_search_speed_rpm
              << " rpm 同时校准 CAN0 四个转向轴；"
              << "各轴负限位立即反向，正限位后独立回中，不在限位等待其他轴。"
              << "故障、通信失败、超速或输出力矩超限均中止。\n";

    for (const auto index : kAllAxes) {
      configure_axis(index);
    }
    auto result = calibrate_all_axes();
    success_ = true;
    return result;
  }

private:
  enum class Phase {negative, releasing_negative, positive, preparing_midpoint, midpoint};
  enum class MidpointStep {configure, mode, shutdown, switch_on, enable};

  struct AxisProgress
  {
    Phase phase{Phase::negative};
    MidpointStep midpoint_step{MidpointStep::configure};
    std::chrono::steady_clock::time_point started;
    std::chrono::steady_clock::time_point previous_time;
    std::int32_t previous_position{0};
    bool midpoint_reported{false};
  };

  static const char * phase_name(Phase phase)
  {
    switch (phase) {
      case Phase::negative: return "负向搜索";
      case Phase::releasing_negative: return "反向离开负限位";
      case Phase::positive: return "正向搜索";
      case Phase::preparing_midpoint: return "切换回中模式";
      case Phase::midpoint: return "回中/保持中位";
    }
    return "未知";
  }

  void configure_axis(std::size_t index)
  {
    require_running(keep_running_);
    auto axis = axes_[index];
    auto result = axis->node()->send_nmt(
      iswv::canopen::NmtCommand::enter_pre_operational);
    require_success(result, std::string(kWheelNames[index]) + " NMT pre-operational failed");
    axis_ready_[index] = true;
    result = axis->drive().transition_to(iswv::cia402::DriveState::switch_on_disabled);
    require_success(result, std::string(kWheelNames[index]) + " 退出使能失败");
    result = axis->command_torque_percent(0.0);
    require_success(result, std::string(kWheelNames[index]) + " 清零力矩失败");
    result = axis->command_motor_velocity_rpm(0.0);
    require_success(result, std::string(kWheelNames[index]) + " 清零速度失败");
    apply_steering_tuning(*axis, config_, index);
    result = axis->configure_default_pdos(config_.motor.pdo);
    require_success(result, std::string(kWheelNames[index]) + " 配置 PDO 失败");
    result = axis->configure_velocity_mode(
      false, std::min(config_.motor.acceleration_rps2, 1.0),
      std::min(config_.motor.deceleration_rps2, 1.0));
    require_success(result, std::string(kWheelNames[index]) + " 配置速度模式失败");
    require_mode(axis, iswv::cia402::OperationMode::profile_velocity);

    iswv::SafetyPolicy safety;
    safety.enabled = true;
    safety.configure_heartbeat_producer = true;
    safety.heartbeat_producer_time = 100ms;
    safety.local_heartbeat_timeout = 350ms;
    safety.timeout_action = iswv::TimeoutAction::quick_stop;
    safety.configure_device_interruption_fault = true;
    result = axis->apply_safety_policy(safety);
    require_success(result, std::string(kWheelNames[index]) + " 配置安全策略失败");
    result = axis->node()->send_nmt(iswv::canopen::NmtCommand::start);
    require_success(result, std::string(kWheelNames[index]) + " NMT start failed");
    enable_axis(axis, index, "初次使能");
  }

  void require_mode(
    const std::shared_ptr<iswv::Axis> & axis, iswv::cia402::OperationMode expected)
  {
    const auto mode_deadline = std::chrono::steady_clock::now() + 2s;
    while (true) {
      require_running(keep_running_);
      const auto mode = axis->node()->read(iswv::objects::mode);
      if (!mode) {
        throw std::runtime_error("无法回读工作模式 0x6060: " + mode.error().message);
      }
      if (mode.value() == static_cast<std::int8_t>(expected)) {
        return;
      }
      if (std::chrono::steady_clock::now() >= mode_deadline) {
        throw std::runtime_error(
                "工作模式 0x6060 回读不匹配: expected=" +
                std::to_string(static_cast<int>(expected)) + ", actual=" +
                std::to_string(static_cast<int>(mode.value())) + "，停止校准");
      }
      std::this_thread::sleep_for(20ms);
    }
  }

  void enable_axis(
    const std::shared_ptr<iswv::Axis> & axis, std::size_t index,
    const char * stage)
  {
    const auto enabled = axis->drive().enable();
    if (enabled) {
      return;
    }

    std::ostringstream message;
    message << stage << " " << kWheelNames[index]
            << " (Node-ID " << static_cast<unsigned>(config_.motor.node_ids[index])
            << ") 失败: " << enabled.error().message;
    const auto refreshed = axis->refresh_state();
    const auto snapshot = axis->state_snapshot();
    message << "; status(0x6041)=0x" << std::hex << std::setw(4)
            << std::setfill('0') << snapshot.status_word
            << ", error_status(0x2601)=0x" << std::setw(4) << snapshot.error_status
            << ", error_status_2(0x2602)=0x" << std::setw(4) << snapshot.error_status_2
            << std::dec;
    if (!refreshed) {
      message << "; 刷新诊断对象失败: " << refreshed.error().message;
    }
    throw std::runtime_error(message.str());
  }

  iswv::cia402::DriveState require_healthy(
    const std::shared_ptr<iswv::Axis> & axis, std::size_t index, bool allow_disabled = false)
  {
    const auto status = axis->drive().read_status();
    const auto error1 = axis->node()->read(iswv::objects::error_status);
    const auto error2 = axis->node()->read(iswv::objects::error_status_2);
    if (!status || !error1 || !error2) {
      std::ostringstream message;
      message << kWheelNames[index] << " (Node-ID "
              << static_cast<unsigned>(config_.motor.node_ids[index])
              << ") 无法读取驱动器保护状态，停止校准";
      if (!status) {message << "; status(0x6041): " << status.error().message;}
      if (!error1) {message << "; error_status(0x2601): " << error1.error().message;}
      if (!error2) {message << "; error_status_2(0x2602): " << error2.error().message;}
      throw std::runtime_error(message.str());
    }
    const auto state = iswv::cia402::decode_state(status.value());
    const bool allowed_disabled = allow_disabled &&
      (state == iswv::cia402::DriveState::switch_on_disabled ||
      state == iswv::cia402::DriveState::ready_to_switch_on ||
      state == iswv::cia402::DriveState::switched_on);
    if ((!allowed_disabled && state != iswv::cia402::DriveState::operation_enabled) ||
      error1.value() != 0 || error2.value() != 0)
    {
      std::ostringstream message;
      message << kWheelNames[index] << " 驱动器非正常使能或存在故障，停止校准: status=0x"
              << std::hex << status.value() << ", error1=0x" << error1.value()
              << ", error2=0x" << error2.value() << std::dec;
      if ((error1.value() & 0x0800U) != 0U) {
        message << "；检测到电机/驱动器过载，疑似硬限位卡滞、机械阻力过大或持续顶住限位";
      }
      throw std::runtime_error(message.str());
    }
    return state;
  }

  void require_ready_axes_healthy()
  {
    for (const auto index : kAllAxes) {
      if (axis_ready_[index]) {
        require_healthy(axes_[index], index);
      }
    }
  }

  SteeringCalibrationResult calibrate_all_axes()
  {
    constexpr auto search_period = 20ms;
    SteeringCalibrationResult result;
    std::array<AxisProgress, 4> progress;
    const TorqueLimitDetector detector(
      config_.calibration_contact_torque_nm, config_.calibration_torque_limit_nm);
    // A PV reversal must decelerate through zero before moving away. Allow its
    // configured ramp plus a bounded settling margin, never the full search wait.
    const double ramp_rps2 = std::min(
        {
          config_.motor.acceleration_rps2, config_.motor.deceleration_rps2, 1.0});
    std::array<double, 4> release_timeout_s{};
    for (const auto index : kAllAxes) {
      release_timeout_s[index] = std::min(
        config_.calibration_timeout_s,
        std::max(1.0, 2.0 * search_motor_rpm_[index] / (60.0 * ramp_rps2) + 0.5));
    }
    const auto release_distance_inc = std::max(1, config_.calibration_position_epsilon_inc);

    for (const auto index : kAllAxes) {
      const auto initial = axes_[index]->node()->read(iswv::objects::actual_position);
      require_success(initial, std::string(kWheelNames[index]) + " 无法读取起始位置");
      progress[index].previous_position = initial.value();
    }
    require_ready_axes_healthy();
    for (const auto index : kAllAxes) {
      require_running(keep_running_);
      require_success(
        axes_[index]->command_velocity_pdo(-search_motor_rpm_[index]),
        std::string(kWheelNames[index]) + " 发送负向校准速度失败");
      progress[index].started = std::chrono::steady_clock::now();
      progress[index].previous_time = progress[index].started;
    }

    auto last_print = std::chrono::steady_clock::now();
    while (true) {
      require_running(keep_running_);
      const auto loop_start = std::chrono::steady_clock::now();
      const bool print_feedback = loop_start - last_print >= 500ms;
      bool all_centered = true;
      for (const auto index : kAllAxes) {
        require_running(keep_running_);
        auto & state = progress[index];
        const auto drive_state = require_healthy(
          axes_[index], index, state.phase == Phase::preparing_midpoint);
        const auto position = axes_[index]->node()->read(iswv::objects::actual_position);
        const auto current = axes_[index]->node()->read(iswv::objects::actual_current);
        require_success(position, std::string(kWheelNames[index]) + " 读取校准位置失败");
        require_success(current, std::string(kWheelNames[index]) + " 读取校准电流失败");
        const auto now = std::chrono::steady_clock::now();
        const double torque_nm = output_torque_nm(config_, modules_[index], current.value());
        // Includes release, return and already-centered axes: residual contact
        // may defer limit detection, but must never bypass the safety ceiling.
        (void)detector.update(torque_nm, false);
        const double elapsed_s = std::chrono::duration<double>(now - state.started).count();
        if (print_feedback) {
          std::cout << "反馈 " << kWheelNames[index] << "：阶段=" << phase_name(state.phase)
                    << "；位置=" << position.value() << " inc；实际电流=" << current.value()
                    << " raw；估算输出力矩=" << torque_nm << " N·m（未计减速效率）\n";
        }

        if (state.phase == Phase::midpoint) {
          const auto error_inc = std::abs(
            static_cast<std::int64_t>(position.value()) - result.midpoints[index]);
          const bool centered = error_inc <= config_.calibration_position_epsilon_inc;
          all_centered = all_centered && centered;
          if (centered && !state.midpoint_reported) {
            state.midpoint_reported = true;
            std::cout << kWheelNames[index] << " 中位已到达：目标=" << result.midpoints[index]
                      << " inc，实际=" << position.value() << " inc，保持并等待其他轴完成\n";
          }
          if (!centered && elapsed_s >= config_.calibration_timeout_s) {
            throw std::runtime_error(std::string(kWheelNames[index]) + " 移动到实时中位超时");
          }
          continue;
        }

        all_centered = false;
        if (state.phase == Phase::preparing_midpoint) {
          if (elapsed_s >= std::min(2.0, config_.calibration_timeout_s)) {
            throw std::runtime_error(std::string(kWheelNames[index]) + " 切换回中模式超时");
          }
          prepare_midpoint(index, state, drive_state, position.value(), result.midpoints[index]);
          continue;
        }
        const double seconds = std::chrono::duration<double>(now - state.previous_time).count();
        const auto delta = static_cast<std::int64_t>(position.value()) - state.previous_position;
        if (seconds > 0.0) {
          const double rpm = std::abs(static_cast<double>(delta)) * 60.0 /
            (modules_[index].steering_encoder_resolution * seconds);
          if (rpm > config_.calibration_speed_rpm) {
            throw std::runtime_error(
                    std::string(kWheelNames[index]) + " 电机实测速度 " + std::to_string(rpm) +
                    " rpm 超过 calibration_speed_rpm=" +
                    std::to_string(config_.calibration_speed_rpm) + "，停止校准");
          }
        }
        state.previous_time = now;
        state.previous_position = position.value();
        const double timeout_s = state.phase == Phase::releasing_negative ?
          release_timeout_s[index] : config_.calibration_timeout_s;
        if (elapsed_s >= timeout_s) {
          throw std::runtime_error(
                  std::string(kWheelNames[index]) + " " + phase_name(state.phase) +
                  "超时；检查硬限位、机械卡滞和电流反馈，停止全部校准轴");
        }

        const bool contact = torque_nm >= config_.calibration_contact_torque_nm;
        if (state.phase == Phase::releasing_negative) {
          const auto moved = static_cast<std::int64_t>(position.value()) -
            result.negative_limits[index];
          if (moved >= release_distance_inc && !contact) {
            state.phase = Phase::positive;
            state.started = now;
            std::cout << kWheelNames[index] << " 已离开负限位，开始检测正限位\n";
          }
        } else if (state.phase == Phase::negative && contact) {
          result.negative_limits[index] = position.value();
          // Send the opposite target in this iteration, before servicing another
          // axis. PV acceleration/deceleration remains active in the drive.
          require_running(keep_running_);
          require_success(
            axes_[index]->command_velocity_pdo(search_motor_rpm_[index]),
            std::string(kWheelNames[index]) + " 负限位反向指令失败");
          state.phase = Phase::releasing_negative;
          state.started = std::chrono::steady_clock::now();
          std::cout << kWheelNames[index] << " 负向限位已检测：位置=" << position.value()
                    << " inc，估算输出力矩=" << torque_nm << " N·m，已下发反向速度\n";
          continue;
        } else if (state.phase == Phase::positive && contact) {
          const auto separation_inc = static_cast<std::int64_t>(position.value()) -
            result.negative_limits[index];
          const double separation_rad = static_cast<double>(separation_inc) * 2.0 * kPi /
            (static_cast<double>(modules_[index].steering_encoder_resolution) *
            modules_[index].steering_gear_ratio);
          if (separation_rad < config_.calibration_min_limit_separation_rad) {
            throw std::runtime_error(
                    std::string(kWheelNames[index]) + " 硬限位异常：正向接触力矩已达到阈值，"
                    "但距上一限位仅 " + std::to_string(separation_rad) +
                    " rad，小于最小间距 " +
                    std::to_string(config_.calibration_min_limit_separation_rad) +
                    " rad；疑似机械卡滞、硬限位位置异常或编码器方向/计数异常，停止校准");
          }
          result.positive_limits[index] = position.value();
          start_midpoint(index, result);
          state.phase = Phase::preparing_midpoint;
          state.started = std::chrono::steady_clock::now();
          continue;
        }

        require_running(keep_running_);
        require_success(
          axes_[index]->command_velocity_pdo(
            state.phase == Phase::negative ? -search_motor_rpm_[index] : search_motor_rpm_[index]),
          std::string(kWheelNames[index]) + " 发送校准速度失败");
      }
      if (all_centered) {
        std::cout << "四个转向轴均已完成校准并保持实时中位。\n";
        return result;
      }
      if (print_feedback) {last_print = loop_start;}
      // Four axes share one CAN bus. Include SDO time; no barrier at either limit
      // or midpoint. This is a scheduling target, not hardware synchronization.
      std::this_thread::sleep_until(loop_start + search_period);
    }
  }

  void start_midpoint(std::size_t index, SteeringCalibrationResult & result)
  {
    require_running(keep_running_);
    auto axis = axes_[index];
    require_success(
      axis->command_motor_velocity_rpm(0.0),
      std::string(kWheelNames[index]) + " 切换位置模式前无法发送零速度");
    require_success(
      axis->drive().write_control(iswv::cia402::control::disable_voltage),
      std::string(kWheelNames[index]) + " 切换位置模式前退出使能失败");
    if (result.negative_limits[index] >= result.positive_limits[index]) {
      throw std::runtime_error(std::string(kWheelNames[index]) + " 硬限位顺序无效");
    }
    result.midpoints[index] = static_cast<std::int32_t>(
      (static_cast<std::int64_t>(result.negative_limits[index]) +
      result.positive_limits[index]) / 2);
    std::cout << kWheelNames[index] << "：负限位=" << result.negative_limits[index]
              << " inc (" << position_to_rad(
      modules_[index], result.negative_limits[index], result.midpoints[index])
              << " rad)，正限位=" << result.positive_limits[index] << " inc ("
              << position_to_rad(
      modules_[index], result.positive_limits[index], result.midpoints[index])
              << " rad)，实时中位=" << result.midpoints[index] << " inc，立即独立回中\n";

  }

  void prepare_midpoint(
    std::size_t index, AxisProgress & progress, iswv::cia402::DriveState state,
    std::int32_t current_position, std::int32_t midpoint)
  {
    auto axis = axes_[index];
    require_running(keep_running_);
    // Poll each state once per outer loop. No mode/enable wait loop may leave
    // the other three searching axes unobserved while this axis changes mode.
    switch (progress.midpoint_step) {
      case MidpointStep::configure:
        if (state != iswv::cia402::DriveState::switch_on_disabled) {return;}
        require_success(
          axis->configure_position_mode(
            config_.motor.profile_velocity_rpm, config_.motor.acceleration_rps2,
            config_.motor.deceleration_rps2),
          std::string(kWheelNames[index]) + " 配置位置模式失败");
        // Remove stale targets while disabled, before the enable sequence.
        require_success(
          axis->node()->write(iswv::objects::target_position, current_position),
          std::string(kWheelNames[index]) + " 回中使能前清除旧位置目标失败");
        progress.midpoint_step = MidpointStep::mode;
        return;
      case MidpointStep::mode: {
          const auto mode = axis->node()->read(iswv::objects::mode);
          require_success(mode, std::string(kWheelNames[index]) + " 回中模式读取失败");
          if (mode.value() != static_cast<std::int8_t>(
              iswv::cia402::OperationMode::profile_position)) {return;}
          require_success(
            axis->drive().write_control(iswv::cia402::control::shutdown),
            std::string(kWheelNames[index]) + " 回中 shutdown 失败");
          progress.midpoint_step = MidpointStep::shutdown;
          return;
        }
      case MidpointStep::shutdown:
        if (state != iswv::cia402::DriveState::ready_to_switch_on) {return;}
        require_success(
          axis->drive().write_control(iswv::cia402::control::switch_on),
          std::string(kWheelNames[index]) + " 回中 switch-on 失败");
        progress.midpoint_step = MidpointStep::switch_on;
        return;
      case MidpointStep::switch_on:
        if (state != iswv::cia402::DriveState::switched_on) {return;}
        require_success(
          axis->drive().write_control(iswv::cia402::control::enable_operation),
          std::string(kWheelNames[index]) + " 回中使能失败");
        progress.midpoint_step = MidpointStep::enable;
        return;
      case MidpointStep::enable:
        if (state != iswv::cia402::DriveState::operation_enabled) {return;}
        require_success(
          axis->command_position_pdo(midpoint, false),
          std::string(kWheelNames[index]) + " 发送实时中位失败");
        progress.phase = Phase::midpoint;
        progress.started = std::chrono::steady_clock::now();
        return;
    }
  }

  void stop_axis(std::size_t index) noexcept
  {
    if (!axes_[index]) {
      return;
    }
    (void)axes_[index]->command_motor_velocity_rpm(0.0);
    (void)axes_[index]->drive().quick_stop();
    (void)axes_[index]->drive().disable();
    axis_ready_[index] = false;
  }

  void stop_all() noexcept
  {
    for (const auto index : kAllAxes) {
      stop_axis(index);
    }
  }

  SteeringConfigFile & config_;
  const std::function<bool()> & keep_running_;
  std::array<SwerveModuleConfig, 4> modules_{};
  std::array<double, 4> search_motor_rpm_{};
  std::array<std::shared_ptr<iswv::Axis>, 4> axes_{};
  std::array<bool, 4> axis_ready_{};
  bool success_{false};
};

}  // namespace

SteeringCalibrationResult calibrate_steering_with_limits(
  iswv::SteeringLayout & steering,
  SteeringConfigFile & config,
  const std::function<bool()> & keep_running)
{
  SteeringCalibrator calibrator(steering, config, keep_running);
  return calibrator.run();
}

std::array<std::int32_t, 4> calibrate_steering(
  iswv::SteeringLayout & steering,
  SteeringConfigFile & config,
  const std::function<bool()> & keep_running)
{
  return calibrate_steering_with_limits(steering, config, keep_running).midpoints;
}

SteeringCalibrationResult calibrate_steering_with_limits(
  SteeringAxes & steering,
  SteeringConfigFile & config,
  const std::function<bool()> & keep_running)
{
  SteeringCalibrator calibrator(steering, config, keep_running);
  return calibrator.run();
}

std::array<std::int32_t, 4> calibrate_steering(
  SteeringAxes & steering,
  SteeringConfigFile & config,
  const std::function<bool()> & keep_running)
{
  return calibrate_steering_with_limits(steering, config, keep_running).midpoints;
}


}  // namespace chassis
