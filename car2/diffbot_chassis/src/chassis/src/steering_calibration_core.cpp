#include "chassis/steering_calibration.hpp"

#include "chassis/torque_limit_detector.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace chassis
{
namespace
{

using namespace std::chrono_literals;

constexpr double kPi = 3.14159265358979323846;
constexpr std::array<const char *, 4> kWheelNames{
  "front_left", "rear_left", "rear_right", "front_right"};
constexpr std::array<std::size_t, 4> kAllAxes{{0, 1, 2, 3}};
using CalibrationPair = std::array<std::size_t, 2>;
constexpr std::array<CalibrationPair, 2> kCalibrationPairs{{
  CalibrationPair{{0, 1}}, CalibrationPair{{2, 3}}}};

double position_to_rad(
  const SteeringConfigFile & config, std::size_t index, std::int32_t position,
  std::int32_t zero_offset)
{
  const auto relative_inc = static_cast<std::int64_t>(position) - zero_offset;
  const double direction = config.motor.inverted[index] ? -1.0 : 1.0;
  return direction * static_cast<double>(relative_inc) * 2.0 * kPi /
         (static_cast<double>(config.motor.encoder_resolution) * config.motor.gear_ratio);
}

double output_torque_nm(const SteeringConfigFile & config, std::int16_t current_raw)
{
  const double current_a = static_cast<double>(current_raw) *
    (config.motor_peak_current_a / 1.414) / 2048.0;
  return std::abs(current_a) * config.motor_torque_constant_nm_per_a * config.motor.gear_ratio;
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
  SteeringCalibrator(
    iswv::SteeringLayout & steering, const SteeringConfigFile & config,
    const std::function<bool()> & keep_running)
  : steering_(steering), config_(config), keep_running_(keep_running),
    search_motor_rpm_(calibration_motor_search_rpm(config))
  {
    validate_chassis_config(config_);
    for (std::size_t index = 0; index < axes_.size(); ++index) {
      axes_[index] = steering_.at(static_cast<iswv::WheelPosition>(index));
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
              << " rpm（电机 " << search_motor_rpm_
              << " rpm）分两组校准 CAN0 转向电机："
              << "前左+后左，然后后右+前右；各轴碰限后单独停下。"
              << "估算输出力矩达到接触阈值即判定限位；"
              << "故障、通信失败、超速或输出力矩超限均中止。\n";

    SteeringCalibrationResult result;

    for (std::size_t pair_index = 0; pair_index < kCalibrationPairs.size(); ++pair_index) {
      const auto & pair = kCalibrationPairs[pair_index];
      std::cout << "开始第 " << pair_index + 1 << " 组："
                << kWheelNames[pair[0]] << " + " << kWheelNames[pair[1]] << "。\n";
      for (const auto index : pair) {
        configure_axis(index);
      }
      seek_limits(pair, -search_motor_rpm_, "负向", result.negative_limits);
      release_velocity_axes(pair);
      seek_limits(
        pair, search_motor_rpm_, "正向", result.positive_limits, result.negative_limits);

      for (const auto index : pair) {
        if (result.negative_limits[index] >= result.positive_limits[index]) {
          throw std::runtime_error(
                  std::string(kWheelNames[index]) +
                  " 硬限位顺序无效，请检查电机方向、编码器方向或机械限位");
        }
        result.midpoints[index] = static_cast<std::int32_t>(
          (static_cast<std::int64_t>(result.negative_limits[index]) +
          result.positive_limits[index]) / 2);
        const auto negative_rad = position_to_rad(
          config_, index, result.negative_limits[index], result.midpoints[index]);
        const auto positive_rad = position_to_rad(
          config_, index, result.positive_limits[index], result.midpoints[index]);
        std::cout << kWheelNames[index] << "：负限位=" << result.negative_limits[index]
                  << " inc (" << std::fixed << std::setprecision(4) << negative_rad << " rad)"
                  << "，正限位=" << result.positive_limits[index] << " inc (" << positive_rad
                  << " rad)"
                  << "，实时中位=" << result.midpoints[index] << " inc。\n";
      }

      move_to_midpoints(pair, result.midpoints);
      std::cout << "第 " << pair_index + 1 << " 组校准完成并保持中位。\n";
    }

    success_ = true;
    return result;
  }

private:
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

  void release_velocity_axes(const CalibrationPair & pair)
  {
    for (const auto index : pair) {
      (void)axes_[index]->command_motor_velocity_rpm(0.0);
      (void)axes_[index]->drive().quick_stop();
      (void)axes_[index]->drive().disable();
      axis_ready_[index] = false;
    }
    std::this_thread::sleep_for(1s);
    for (const auto index : pair) {
      configure_axis(index);
    }
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

  void require_healthy(const std::shared_ptr<iswv::Axis> & axis, std::size_t index)
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
    if (iswv::cia402::decode_state(status.value()) !=
      iswv::cia402::DriveState::operation_enabled || error1.value() != 0 || error2.value() != 0)
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
  }

  void require_ready_axes_healthy()
  {
    for (const auto index : kAllAxes) {
      if (axis_ready_[index]) {
        require_healthy(axes_[index], index);
      }
    }
  }

  void seek_limits(
    const CalibrationPair & pair, double motor_rpm, const char * direction,
    std::array<std::int32_t, 4> & positions,
    std::optional<std::array<std::int32_t, 4>> previous_limits = std::nullopt)
  {
    constexpr auto search_period = 20ms;
    for (const auto index : pair) {
      std::cout << kWheelNames[index] << " 开始向" << direction
                << "发送电机速度 " << motor_rpm << " rpm（输出轴 "
                << motor_rpm / config_.motor.gear_ratio << " rpm）。\n";
    }

    const auto start = std::chrono::steady_clock::now();
    auto last_print = start;
    std::array<std::chrono::steady_clock::time_point, 4> previous_times;
    previous_times.fill(start);
    std::array<std::int32_t, 4> previous_positions{};
    std::array<bool, 4> detected{};
    std::size_t detected_count = 0;
    for (const auto index : pair) {
      const auto initial = axes_[index]->node()->read(iswv::objects::actual_position);
      if (!initial) {
        throw std::runtime_error(
                std::string(kWheelNames[index]) + " 无法读取起始位置，停止校准");
      }
      previous_positions[index] = initial.value();
    }

    std::array<std::optional<TorqueLimitDetector>, 4> detectors;
    for (const auto index : pair) {
      detectors[index].emplace(
        config_.calibration_contact_torque_nm, config_.calibration_torque_limit_nm);
    }

    while (true) {
      require_running(keep_running_);
      const auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(now - start).count() >= config_.calibration_timeout_s) {
        std::ostringstream message;
        message << "硬限位检测超时：" << direction << "方向在 "
                << config_.calibration_timeout_s << " 秒内未检测到";
        bool first = true;
        for (const auto index : pair) {
          if (!detected[index]) {
            message << (first ? " " : ", ") << kWheelNames[index];
            first = false;
          }
        }
        message << " 的接触力矩，请检查硬限位、机械连接、电流反馈和力矩阈值，停止校准";
        throw std::runtime_error(message.str());
      }

      require_ready_axes_healthy();
      for (const auto index : pair) {
        if (detected[index]) {
          continue;
        }
        const auto command = axes_[index]->command_velocity_pdo(motor_rpm);
        require_success(command, std::string(kWheelNames[index]) + " 发送校准速度失败");
      }

      for (const auto index : pair) {
        if (detected[index]) {
          continue;
        }
        const auto position_result = axes_[index]->node()->read(iswv::objects::actual_position);
        const auto current_result = axes_[index]->node()->read(iswv::objects::actual_current);
        if (!position_result || !current_result) {
          throw std::runtime_error(
                  std::string(kWheelNames[index]) + " 读取限位反馈失败，停止校准");
        }
        const auto current_position = position_result.value();
        const auto torque_nm = output_torque_nm(config_, current_result.value());
        const auto separation_inc = previous_limits ?
          std::abs(
          static_cast<std::int64_t>(current_position) - (*previous_limits)[index]) : 0;
        const double separation_rad = static_cast<double>(separation_inc) * 2.0 * kPi /
          (static_cast<double>(config_.motor.encoder_resolution) * config_.motor.gear_ratio);
        const bool separation_reached = !previous_limits ||
          separation_rad >= config_.calibration_min_limit_separation_rad;
        const auto sample_time = std::chrono::steady_clock::now();
        const bool contact_reached = detectors[index]->update(torque_nm, separation_reached);
        if (previous_limits && !separation_reached &&
          std::abs(torque_nm) >= config_.calibration_contact_torque_nm)
        {
          std::ostringstream message;
          message << kWheelNames[index] << " 硬限位异常：" << direction
                  << "接触力矩 " << std::fixed << std::setprecision(3) << torque_nm
                  << " N·m 已达到阈值 " << config_.calibration_contact_torque_nm
                  << " N·m，但距上一限位仅 " << separation_rad
                  << " rad，小于最小间距 "
                  << config_.calibration_min_limit_separation_rad
                  << " rad；疑似机械卡滞、硬限位位置异常或编码器方向/计数异常，停止校准";
          throw std::runtime_error(message.str());
        }
        const auto seconds = std::chrono::duration<double>(
          sample_time - previous_times[index]).count();
        const auto delta = static_cast<std::int64_t>(current_position) -
          previous_positions[index];
        if (seconds > 0.0) {
          const auto rpm = std::abs(static_cast<double>(delta)) * 60.0 /
            (config_.motor.encoder_resolution * seconds);
          if (rpm > config_.calibration_speed_rpm) {
            throw std::runtime_error(
                    std::string(kWheelNames[index]) + " 电机实测速度 " + std::to_string(rpm) +
                    " rpm 超过 calibration_speed_rpm=" +
                    std::to_string(config_.calibration_speed_rpm) + "，停止校准");
          }
        }
        previous_times[index] = sample_time;
        previous_positions[index] = current_position;
        if (contact_reached) {
          const auto stopped = axes_[index]->command_motor_velocity_rpm(0.0);
          require_success(stopped, std::string(kWheelNames[index]) + " 无法撤销校准速度");
          positions[index] = current_position;
          detected[index] = true;
          ++detected_count;
          std::cout << kWheelNames[index] << " " << direction << "限位已检测：位置="
                    << positions[index] << " inc，估算输出力矩=" << std::fixed
                    << std::setprecision(3) << torque_nm << " N·m";
          if (previous_limits) {
            std::cout << "，距上一限位=" << separation_rad << " rad";
          }
          std::cout << "，该轴已停止等待其他轴\n";
        }
      }
      if (detected_count == pair.size()) {
        std::this_thread::sleep_for(200ms);
        require_running(keep_running_);
        require_ready_axes_healthy();
        return;
      }
      if (now - last_print >= 500ms) {
        print_feedback_for_undetected(pair, detected);
        last_print = now;
      }
      // Include synchronous feedback reads in the target period. If CAN reads
      // take longer, continue immediately; this is not a hard real-time bound.
      std::this_thread::sleep_until(now + search_period);
    }
  }

  void move_to_midpoints(
    const CalibrationPair & pair, const std::array<std::int32_t, 4> & midpoints)
  {
    for (const auto index : pair) {
      auto result = axes_[index]->command_motor_velocity_rpm(0.0);
      require_success(result, std::string(kWheelNames[index]) + " 切换位置模式前无法发送零速度");
      result = axes_[index]->drive().transition_to(iswv::cia402::DriveState::switch_on_disabled);
      require_success(result, std::string(kWheelNames[index]) + " 切换位置模式前退出使能失败");
    }

    for (const auto index : pair) {
      auto result = axes_[index]->configure_position_mode(
        config_.motor.profile_velocity_rpm, config_.motor.acceleration_rps2,
        config_.motor.deceleration_rps2);
      require_success(result, std::string(kWheelNames[index]) + " 配置位置模式失败");
      require_mode(axes_[index], iswv::cia402::OperationMode::profile_position);
      enable_axis(axes_[index], index, "位置模式使能");
      require_healthy(axes_[index], index);
    }

    for (const auto index : pair) {
      const auto result = axes_[index]->command_position_pdo(midpoints[index], false);
      require_success(result, std::string(kWheelNames[index]) + " 发送实时中位失败");
    }

    const auto start = std::chrono::steady_clock::now();
    std::array<bool, 4> reached{};
    while (true) {
      require_running(keep_running_);
      require_ready_axes_healthy();
      bool all_reached = true;
      for (const auto index : pair) {
        const auto actual = axes_[index]->node()->read(iswv::objects::actual_position);
        if (!actual) {
          throw std::runtime_error(
                  std::string(kWheelNames[index]) + " 读取中位运动位置失败: " +
                  actual.error().message);
        }
        const auto error_inc = std::abs(
          static_cast<std::int64_t>(actual.value()) - midpoints[index]);
        if (!reached[index] && error_inc <= config_.calibration_position_epsilon_inc) {
          reached[index] = true;
          std::cout << kWheelNames[index] << " 中位已到达：目标=" << midpoints[index]
                    << " inc，实际=" << actual.value() << " inc，误差=" << error_inc << " inc\n";
        }
        all_reached = all_reached && error_inc <= config_.calibration_position_epsilon_inc;
      }
      if (all_reached) {
        return;
      }
      const auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(now - start).count() >= config_.calibration_timeout_s) {
        throw std::runtime_error("当前转向电机组移动到实时中位超时");
      }
      std::this_thread::sleep_for(100ms);
    }
  }

  void print_feedback_for_undetected(
    const CalibrationPair & pair, const std::array<bool, 4> & detected)
  {
    for (const auto index : pair) {
      if (detected[index]) {
        continue;
      }
      const auto position = axes_[index]->node()->read(iswv::objects::actual_position);
      const auto current = axes_[index]->node()->read(iswv::objects::actual_current);
      if (!position || !current) {
        continue;
      }
      std::cout << "反馈 " << kWheelNames[index]
                << "：位置=" << position.value() << " inc"
                << "；实际电流=" << current.value() << " raw"
                << "；估算输出力矩=" << output_torque_nm(config_, current.value())
                << " N·m（未计减速效率）\n";
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

  iswv::SteeringLayout & steering_;
  const SteeringConfigFile & config_;
  const std::function<bool()> & keep_running_;
  double search_motor_rpm_{0.0};
  std::array<std::shared_ptr<iswv::Axis>, 4> axes_{};
  std::array<bool, 4> axis_ready_{};
  bool success_{false};
};

}  // namespace

SteeringCalibrationResult calibrate_steering_with_limits(
  iswv::SteeringLayout & steering,
  const SteeringConfigFile & config,
  const std::function<bool()> & keep_running)
{
  SteeringCalibrator calibrator(steering, config, keep_running);
  return calibrator.run();
}

std::array<std::int32_t, 4> calibrate_steering(
  iswv::SteeringLayout & steering,
  const SteeringConfigFile & config,
  const std::function<bool()> & keep_running)
{
  return calibrate_steering_with_limits(steering, config, keep_running).midpoints;
}

}  // namespace chassis
