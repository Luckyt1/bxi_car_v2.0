#include "chassis/control/chassis_control.h"
#include "chassis/motor/bxi_pci_transport.h"
#include "chassis/motor/iswv_motor.h"
#include "chassis/swerve_kinematics.hpp"
#include "chassis/steering_tuning.hpp"
#include "chassis/torque_limit_detector.hpp"
#include "iswv/iswv/object_dictionary.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>
#include <mutex>
#include <iostream>
#include <iomanip>
#include <sstream>

namespace chassis
{
namespace
{
using namespace std::chrono_literals;
constexpr double kPi = 3.14159265358979323846;
template<typename T>
void require_success(const iswv::Result<T> & result, const std::string & operation)
{
  if (!result) {throw std::runtime_error(operation + ": " + result.error().message);}
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

class ChassisController::Impl
{
public:
  static constexpr std::size_t kWheelCount = 4;
  static constexpr std::array<const char *, 4> kNames{
    "front_left", "rear_left", "rear_right", "front_right"};
  struct SteeringFeedback
  {
    std::mutex mutex;
    std::array<std::array<iswv::canopen::PdoEvent, 2>, 4> frames{};
  };
  Impl(SteeringConfigFile config, Hardware hardware)
  : config_(std::move(config)), hardware_(std::move(hardware))
  {
    validate_chassis_config(config_);
    modules_ = resolved_modules(config_);
    drive_max_output_rpm_ = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      const auto & module = modules_[i];
      const double max_speed = module.drive_max_motor_rpm / module.drive_gear_ratio *
        (2.0 * kPi * module.wheel_radius_m) / 60.0;
      kinematics_[i] = {module.x_m, module.y_m,
        {module.steering_min_rad + module.soft_margin_rad,
          module.steering_max_rad - module.soft_margin_rad}, module.heading_offset_rad, max_speed};
      max_module_radius_m_ = std::max(max_module_radius_m_, std::hypot(module.x_m, module.y_m));
      drive_max_output_rpm_ = std::min(
        drive_max_output_rpm_,
        max_speed * 60.0 / (2.0 * kPi * modules_[0].wheel_radius_m));
    }
    wheel_diameter_m_ = 2.0 * modules_[0].wheel_radius_m;
    if (!hardware_.open) {
      hardware_.open = [](unsigned int bus) {return std::make_shared<BxiPciTransport>(bus);};
      hardware_.power = [this](bool enabled) {
          const auto transport = std::dynamic_pointer_cast<BxiPciTransport>(transports_[0]);
          if (transport) {require_success(transport->set_motor_power(enabled), "motor power");}
        };
    }
  }

  ~Impl() {shutdown();}

  void initialize(const std::function<bool()> & keep_running)
  {
    if (initialized_ || state_ != State::uninitialized) {
      throw std::logic_error("chassis already initialized");
    }
    try {
      if (!keep_running()) {throw std::runtime_error("chassis initialization cancelled");}
      for (unsigned int bus = 0; bus < transports_.size(); ++bus) {
        transports_[bus] = hardware_.open(bus);
        if (!transports_[bus] || !transports_[bus]->is_open()) {
          throw std::runtime_error("cannot open CAN" + std::to_string(bus));
        }
        masters_[bus] = std::make_unique<iswv::canopen::CanopenMaster>(transports_[bus]);
      }
      // Construct every axis before power-on, so partial failures can stop all of them.
      steering_ = std::make_unique<SteeringAxes>(*masters_[0], config_);
      for (std::size_t i = 0; i < kWheelCount; ++i) {
        for (std::size_t slot = 0; slot < 2; ++slot) {
          steering_subscriptions_[i * 2 + slot] =
            steering_->at(static_cast<iswv::WheelPosition>(i))->node()->on_tpdo(
            slot == 0 ? 1 : 3,
            [cache = steering_feedback_, i, slot](const iswv::canopen::PdoEvent & event) {
              if (event.size != (slot == 0 ? 6 : 8)) {return;}
              std::lock_guard<std::mutex> lock(cache->mutex);
              cache->frames[i][slot] = event;
            });
        }
      }
      for (std::size_t i = 0; i < kWheelCount; ++i) {
        auto axis_config = iswv::AxisConfiguration::manual_travel(config_.drive_node_ids[i]);
        axis_config.encoder_resolution = modules_[i].drive_encoder_resolution;
        axis_config.gear_ratio = modules_[i].drive_gear_ratio;
        axis_config.wheel_diameter_m = 2.0 * modules_[i].wheel_radius_m;
        drive_wheels_[i] = std::make_unique<IswvDrive>(
          *masters_[config_.drive_can_buses[i]], axis_config);
      }
      power_attempted_ = true;
      if (hardware_.power) {hardware_.power(true);}
      const auto deadline = std::chrono::steady_clock::now() + hardware_.power_settle;
      while (std::chrono::steady_clock::now() < deadline) {
        if (!keep_running()) {throw std::runtime_error("chassis initialization cancelled");}
        std::this_thread::sleep_for(20ms);
      }
      for (auto & wheel : drive_wheels_) {wheel->stop();}
      if (!keep_running()) {throw std::runtime_error("chassis initialization cancelled");}
      initialized_ = true;
    } catch (...) {fail(); throw;}
  }

  void calibrate(const std::function<bool()> & keep_running)
  {
    if (!initialized_ || state_ != State::uninitialized) {
      throw std::logic_error("calibration requires an initialized, unused chassis");
    }
    state_ = State::calibrating;
    config_.steering_calibrated = false;
    try {
      const auto calibration = calibrate_steering_with_limits(*steering_, config_, keep_running);
      zero_offsets_ = calibration.midpoints;
      for (std::size_t i = 0; i < kWheelCount; ++i) {
        const auto & module = modules_[i];
        const double counts_per_rad = module.steering_encoder_resolution *
          module.steering_gear_ratio / (2.0 * kPi);
        const double first = module.steering_sign *
          (static_cast<std::int64_t>(calibration.negative_limits[i]) - zero_offsets_[i]) /
          counts_per_rad;
        const double second = module.steering_sign *
          (static_cast<std::int64_t>(calibration.positive_limits[i]) - zero_offsets_[i]) /
          counts_per_rad;
        const double lower = std::max(module.steering_min_rad, std::min(first, second)) +
          module.soft_margin_rad;
        const double upper = std::min(module.steering_max_rad, std::max(first, second)) -
          module.soft_margin_rad;
        if (lower >= upper || lower > 0.0 || upper < 0.0) {
          throw std::runtime_error(
                  std::string(
                    kNames[i]) +
                  " calibration/configured limits have no usable midpoint after margin");
        }
        const double raw_a = zero_offsets_[i] + module.steering_sign * lower * counts_per_rad;
        const double raw_b = zero_offsets_[i] + module.steering_sign * upper * counts_per_rad;
        const auto lo = static_cast<std::int64_t>(std::ceil(std::min(raw_a, raw_b)));
        const auto hi = static_cast<std::int64_t>(std::floor(std::max(raw_a, raw_b)));
        if (lo >= hi || lo > zero_offsets_[i] || hi < zero_offsets_[i] ||
          lo<std::numeric_limits<std::int32_t>::min() ||
          hi> std::numeric_limits<std::int32_t>::max())
        {
          throw std::runtime_error(
                  std::string(kNames[i]) +
                  " effective encoder limits are invalid");
        }
        safe_lower_inc_[i] = static_cast<std::int32_t>(lo);
        safe_upper_inc_[i] = static_cast<std::int32_t>(hi);
        const double joint_a = module.steering_sign * (lo - zero_offsets_[i]) / counts_per_rad;
        const double joint_b = module.steering_sign * (hi - zero_offsets_[i]) / counts_per_rad;
        kinematics_[i].limits = {std::min(joint_a, joint_b), std::max(joint_a, joint_b)};
        std::cout << kNames[i] << " runtime steering range=[" << kinematics_[i].limits.lower
                  << ", " << kinematics_[i].limits.upper << "] rad, safe encoder=["
                  << lo << ", " << hi << "] inc\n";
      }
      last_angles_.fill(0.0);
      if (!keep_running()) {throw std::runtime_error("chassis calibration cancelled");}
      const auto steering_feedback_deadline = std::chrono::steady_clock::now() + 250ms;
      while (true) {
        if (!keep_running()) {throw std::runtime_error("initial steering feedback wait cancelled");}
        try {
          if (!steering_ready_for_drive()) {
            throw std::runtime_error("steering midpoint not reached");
          }
          break;
        } catch (const std::runtime_error &) {
          if (std::chrono::steady_clock::now() >= steering_feedback_deadline) {throw;}
        }
        std::this_thread::sleep_for(5ms);
      }
      for (std::size_t index = 0; index < kWheelCount; ++index) {
        if (!keep_running()) {throw std::runtime_error("chassis initialization cancelled");}
        try {
          drive_wheels_[index]->enable_rpm(
            modules_[index].drive_acceleration_m_s2 * 60.0 /
            (2.0 * kPi * modules_[index].wheel_radius_m),
            modules_[index].drive_deceleration_m_s2 * 60.0 /
            (2.0 * kPi * modules_[index].wheel_radius_m), config_, index);
        } catch (const std::exception & error) {
          throw std::runtime_error(
                  std::string(kNames[index]) + " CAN" +
                  std::to_string(config_.drive_can_buses[index]) + " Node-ID " +
                  std::to_string(config_.drive_node_ids[index]) + ": " + error.what());
        }
      }
      // Wait for initial process data while travel targets remain zero.
      const auto feedback_deadline = std::chrono::steady_clock::now() + 250ms;
      while (true) {
        if (!keep_running()) {throw std::runtime_error("initial feedback wait cancelled");}
        try {
          for (std::size_t i = 0; i < kWheelCount; ++i) {(void)drive_feedback(i);}
          break;
        } catch (const std::runtime_error &) {
          if (std::chrono::steady_clock::now() >= feedback_deadline) {throw;}
        }
        std::this_thread::sleep_for(5ms);
      }
      config_.zero_offset_inc = zero_offsets_;
      config_.steering_calibrated = true;
      last_control_time_ = std::chrono::steady_clock::now();
      state_ = State::ready;
    } catch (...) {fail(); throw;}
  }

  void require_ready() const
  {
    if (state_ == State::uninitialized || state_ == State::calibrating ||
      state_ == State::faulted)
    {
      throw std::logic_error("chassis is not ready; initialize and calibrate first");
    }
  }

  void forward(double rpm)
  {
    require_ready();
    try {
      if (!std::isfinite(rpm) || rpm <= 0.0 || rpm > drive_max_output_rpm_) {
        throw std::invalid_argument("forward RPM must be positive and within drive_max_output_rpm");
      }
      command_ = {rpm * kPi * wheel_diameter_m_ / 60.0, 0.0, 0.0};
      state_ = State::forward_wait;
      control_once();
    } catch (...) {fail(); throw;}
  }

  void set_velocity(double x, double y, double yaw)
  {
    require_ready();
    try {
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
        !std::isfinite(std::hypot(x, y) + std::abs(yaw) * max_module_radius_m_))
      {
        throw std::invalid_argument("chassis velocity must be finite and representable");
      }
      command_ = {x, y, yaw};
      state_ = State::commanded;
    } catch (...) {fail(); throw;}
  }

  void update()
  {
    require_ready();
    try {
      control_once();
    } catch (...) {fail(); throw;}
  }

  void stop()
  {
    require_ready();
    try {
      command_ = {};
      zero_command_active_ = false;
      command_zero_and_monitor();
      state_ = State::stopped;
    } catch (...) {fail(); throw;}
  }

  void shutdown() noexcept
  {
    if (shutdown_) {return;}
    shutdown_ = true;
    stop_drive_wheels();
    // Destroy motor guards while the device is still powered and responsive.
    for (auto & wheel : drive_wheels_) {wheel.reset();}
    stop_drives();
    steering_.reset();
    if (power_attempted_ && hardware_.power) {
      try {hardware_.power(false);} catch (...) {}
    }
  }

  void fail() noexcept
  {
    state_ = State::faulted;
    command_ = {};
    config_.steering_calibrated = false;
    shutdown();
  }

  iswv::Result<void> command_angle(
    const std::shared_ptr<iswv::Axis> & axis, double angle_rad, std::size_t index)
  {
    auto increments = axis->steering_radians_to_inc(angle_rad * modules_[index].steering_sign);
    if (!increments) {return iswv::Result<void>::failure(increments.error());}
    const auto target = static_cast<std::int64_t>(increments.value()) + zero_offsets_[index];
    if (target < safe_lower_inc_[index] || target > safe_upper_inc_[index]) {
      return iswv::Result<void>::failure(
        iswv::ErrorCode::invalid_argument, "steering target outside calibrated safe encoder range");
    }
    return axis->command_position_pdo(static_cast<std::int32_t>(target), false);
  }

  // 执行一轮舵轮控制：读取反馈 → 运动学解算 → 转向输出 → 行走限速与输出。
  // 检查或下发失败时抛出异常，由上层调用处理故障。
  void control_once()
  {
    // 确认转向轴已使能、无故障且反馈未过期，再取得相对标定中位的实际转角（rad）。
    check_steering_health();
    const auto actual = actual_steering_angles();
    const auto now = std::chrono::steady_clock::now();
    // 用实际周期计算本轮允许的变化量；最多按 50 ms 计算，避免调度延迟造成指令突跳。
    const double dt = std::clamp(
      std::chrono::duration<double>(now - last_control_time_).count(), 0.0, 0.05);
    last_control_time_ = now;
    // 全零指令立即清除旋转平滑量；运动时限制整车角速度的变化率（rad/s²）。
    const bool idle = command_.x == 0.0 && command_.y == 0.0 && command_.yaw == 0.0;
    smoothed_yaw_ = idle ? 0.0 : slew_towards(
      smoothed_yaw_, command_.yaw, config_.command_yaw_acceleration_rad_s2 * dt);
    // 将整车速度 x、y（m/s）和 yaw（rad/s）换算成各轮目标角度与线速度。
    // 解算考虑各轮转角范围、速度上限，并利用上一轮目标减少等效转向方案来回切换。
    const SwerveOptions swerve_options{
      config_.swerve_speed_epsilon_m_s, config_.swerve_branch_hysteresis_rad};
    const auto solution = solve_bounded_swerve(
      command_.x, command_.y, smoothed_yaw_, kinematics_, actual,
      swerve_options, previous_targets_);
    if (!solution) {
      throw std::runtime_error(
              std::string(kNames[solution.wheel]) + " bounded swerve: " + solution.reason);
    }
    // 保存的是解算目标；last_angles_ 则记录经过转向限速后实际下发的角度。
    previous_targets_ = solution.targets;
    double max_error = 0.0;
    std::array<double, kWheelCount> goals{};
    for (std::size_t index = 0; index < kWheelCount; ++index) {
      const auto & target = solution.targets[index];
      // 对齐误差按最终目标与实际反馈计算，最大值供 pause_all 策略判断。
      const double error = std::abs(target.angle_rad - actual[index]);
      max_error = std::max(max_error, error);
      // 限制每轮转向指令的变化速度；变化小于阈值时不重复下发。
      const double angle = slew_towards(
        last_angles_[index], target.angle_rad, modules_[index].steering_rate_limit_rad_s * dt);
      if (std::abs(angle - last_angles_[index]) > 1e-6) {
        const auto result = command_angle(
          steering_->at(static_cast<iswv::WheelPosition>(index)), angle, index);
        if (!result) {
          throw std::runtime_error(
                  std::string(
                    kNames[index]) + " steering command: " + result.error().message);
        }
        last_angles_[index] = angle;
      }
      // none 策略保留解算速度；cosine 按转向误差的余弦降速，误差达到 90° 时置零。
      double speed = target.speed_m_s;
      if (config_.drive_alignment_policy == "cosine") {
        speed *= error >= kPi / 2.0 ? 0.0 : std::cos(error);
      }
      // 轮缘线速度转为车轮输出轴 RPM；减速比由 IswvDrive 内部处理。
      // 安装方向相反的行走电机需要翻转指令符号。
      goals[index] = speed * 60.0 / (2.0 * kPi * modules_[index].wheel_radius_m);
      if (modules_[index].drive_inverted) {goals[index] = -goals[index];}
    }
    // pause_all：任一轮偏差超过暂停阈值就暂停全部行走轮，全部回到容差内才恢复。
    // 暂停与恢复分别使用各自阈值，alignment_paused_ 保存跨周期的暂停状态。
    if (config_.drive_alignment_policy == "pause_all") {
      if (max_error > config_.drive_alignment_pause_rad) {
        alignment_paused_ = true;
      } else if (max_error <= config_.drive_steering_tolerance_rad) {alignment_paused_ = false;}
      if (alignment_paused_) {goals.fill(0.0);}
    }
    if (std::all_of(goals.begin(), goals.end(), [](double value) {return value == 0.0;})) {
      // 所有行走目标为零时，下发零速并监测停车反馈，减速由驱动器配置执行。
      // 本轮转向指令已在前面处理，停车不会阻止转向继续跟踪目标。
      command_zero_and_monitor();
      return;
    }
    // 恢复非零行走控制，退出零速监测状态，并检查行走轴反馈。
    zero_command_active_ = false;
    check_drive_wheel_health();
    // 任一轮目标与上次下发转速异号时，先让所有轮的指令向零减速，再进入反向。
    // 此处比较的是指令转速，不代表实际车轮已停止。
    bool reversing = false;
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      reversing = reversing || goals[i] * last_drive_rpm_[i] < 0.0;
    }
    if (reversing) {goals.fill(0.0);}
    // 计算四轮共用的插值比例：取各轮允许比例的最小值，满足每轮加减速度限制。
    // 按转速绝对值是增大还是减小选择加速/减速参数，并将 m/s² 换算成 RPM/s。
    double fraction = 1.0;
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      const double delta = std::abs(goals[i] - last_drive_rpm_[i]);
      const double acceleration = std::abs(goals[i]) > std::abs(last_drive_rpm_[i]) ?
        modules_[i].drive_acceleration_m_s2 : modules_[i].drive_deceleration_m_s2;
      const double rate = acceleration * 60.0 / (2.0 * kPi * modules_[i].wheel_radius_m);
      if (delta > 0.0) {fraction = std::min(fraction, rate * dt / delta);}
    }
    // 从上次下发转速向本轮目标推进相同比例；比例达到 1 时直接取目标值。
    std::array<double, kWheelCount> drive_rpm{};
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      drive_rpm[i] = last_drive_rpm_[i] + fraction * (goals[i] - last_drive_rpm_[i]);
      if (fraction >= 1.0) {drive_rpm[i] = goals[i];}
    }
    // 下发行走转速并更新历史指令；健康检查内部按设定时间间隔执行。
    command_drive_wheels(drive_rpm);
    check_drive_wheel_health();
  }

  bool steering_ready_for_drive()
  {
    check_steering_health();
    const auto actual = actual_steering_angles();
    for (std::size_t index = 0; index < kWheelCount; ++index) {
      if (std::abs(actual[index] - last_angles_[index]) > config_.drive_steering_tolerance_rad) {
        return false;
      }
    }
    return true;
  }

  std::array<std::array<iswv::canopen::PdoEvent, 2>, 4> steering_snapshot()
  {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(steering_feedback_->mutex);
    for (std::size_t index = 0; index < kWheelCount; ++index) {
      for (std::size_t slot = 0; slot < 2; ++slot) {
        const auto & frame = steering_feedback_->frames[index][slot];
        if (frame.received_at == std::chrono::steady_clock::time_point{} ||
          now - frame.received_at > 250ms)
        {
          throw std::runtime_error(
                  std::string(kNames[index]) + " CAN0 Node-ID " +
                  std::to_string(config_.motor.node_ids[index]) + " TPDO" +
                  std::to_string(slot == 0 ? 1 : 3) + " missing or older than 250 ms");
        }
      }
    }
    return steering_feedback_->frames;
  }

  std::array<double, 4> actual_steering_angles()
  {
    const auto frames = steering_snapshot();
    std::array<double, 4> result{};
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      const double counts_per_rad = modules_[i].steering_encoder_resolution *
        modules_[i].steering_gear_ratio / (2.0 * kPi);
      const auto position = iswv::canopen::decode_little_endian<std::int32_t>(
        frames[i][0].data.data() + 2, 4).value();
      if (position < safe_lower_inc_[i] || position > safe_upper_inc_[i]) {
        throw std::runtime_error(
                std::string(kNames[i]) + " actual steering outside safe limits: " +
                std::to_string(position) + " inc, allowed [" + std::to_string(safe_lower_inc_[i]) +
                ", " + std::to_string(safe_upper_inc_[i]) + "]");
      }
      result[i] = modules_[i].steering_sign *
        (static_cast<std::int64_t>(position) - zero_offsets_[i]) / counts_per_rad;
    }
    return result;
  }

  void check_steering_health()
  {
    const auto frames = steering_snapshot();
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      const auto status = iswv::canopen::decode_little_endian<std::uint16_t>(
        frames[i][0].data.data(), 2).value();
      const auto error1 = iswv::canopen::decode_little_endian<std::uint16_t>(
        frames[i][1].data.data() + 4, 2).value();
      const auto error2 = iswv::canopen::decode_little_endian<std::uint16_t>(
        frames[i][1].data.data() + 6, 2).value();
      if (iswv::cia402::decode_state(status) != iswv::cia402::DriveState::operation_enabled ||
        error1 != 0 || error2 != 0)
      {
        std::ostringstream message;
        message << kNames[i] << " CAN0 steering fault: status=0x" << std::hex << status
                << " error1=0x" << error1 << " error2=0x" << error2;
        throw std::runtime_error(
                message.str());
      }
    }
  }

  void command_drive_wheels(
    const std::array<double, kWheelCount> & targets, bool force = false)
  {
    for (std::size_t index = 0; index < kWheelCount; ++index) {
      if (!force && std::abs(targets[index] - last_drive_rpm_[index]) < 1e-4) {continue;}
      drive_wheels_[index]->set_velocity_pdo_rpm(targets[index]);
      last_drive_rpm_[index] = targets[index];
    }
  }

  void check_drive_wheel_health()
  {
    const auto now = std::chrono::steady_clock::now();
    if (now < next_drive_health_check_) {return;}
    for (std::size_t index = 0; index < kWheelCount; ++index) {
      const double actual_rpm = drive_feedback(index);
      if (!std::isfinite(actual_rpm) ||
        std::abs(actual_rpm) >
        (modules_[index].drive_max_motor_rpm / modules_[index].drive_gear_ratio) * 1.25 + 0.1)
      {
        throw std::runtime_error(
                "drive wheel " + std::to_string(index) + " exceeded the RPM safety limit");
      }
    }
    next_drive_health_check_ = now + 20ms;
  }

  void command_zero_and_monitor()
  {
    const auto now = std::chrono::steady_clock::now();
    if (!zero_command_active_) {
      zero_command_active_ = true;
      next_drive_health_check_ = now;
      next_zero_refresh_ = now;
      stop_window_ = {};
      stop_report_start_ = now;
      next_stop_report_ = now + 1s;
    }
    // A cached send is not an acknowledgement from the drive. Refresh zero
    // targets at rest so one lost RPDO cannot leave a previous speed active.
    if (now >= next_zero_refresh_) {
      command_drive_wheels({0.0, 0.0, 0.0, 0.0}, true);
      next_zero_refresh_ = now + 100ms;
    }
    if (now < next_drive_health_check_) {return;}

    std::array<double, kWheelCount> measured_rpm{};
    for (std::size_t index = 0; index < kWheelCount; ++index) {
      const auto sample = drive_feedback_sample(index);
      const double actual_rpm = sample.output_rpm;
      if (!std::isfinite(actual_rpm)) {
        throw std::runtime_error(
                "drive wheel " + std::to_string(index) + " returned non-finite RPM");
      }
      measured_rpm[index] = actual_rpm;
      auto & window = stop_window_[index];
      window.min_rpm = std::min(window.min_rpm, actual_rpm);
      window.max_rpm = std::max(window.max_rpm, actual_rpm);
      window.min_position = std::min(window.min_position, sample.position_inc);
      window.max_position = std::max(window.max_position, sample.position_inc);
      window.peak_current_raw = std::max(
        window.peak_current_raw, std::abs(static_cast<int>(sample.current_raw)));
      ++window.samples;
    }

    if (now >= next_stop_report_) {
      std::ostringstream message;
      message << "STOP_FEEDBACK window_ms=" <<
        std::chrono::duration_cast<std::chrono::milliseconds>(now - stop_report_start_).count()
              << std::fixed << std::setprecision(4);
      for (std::size_t index = 0; index < kWheelCount; ++index) {
        const auto & window = stop_window_[index];
        message << "; " << kNames[index] << " CAN" << config_.drive_can_buses[index]
                << " Node-ID " << static_cast<unsigned>(config_.drive_node_ids[index])
                << " command_rpm=" << last_drive_rpm_[index]
                << " actual_rpm=" << measured_rpm[index]
                << " rpm_range=[" << window.min_rpm << ',' << window.max_rpm << ']'
                << " position_span_inc=" <<
          static_cast<std::int64_t>(window.max_position) - window.min_position
                << " current_peak_raw=" << window.peak_current_raw
                << " samples=" << window.samples;
      }
      stop_diagnostic_ = message.str();
      stop_window_ = {};
      stop_report_start_ = now;
      next_stop_report_ = now + 1s;
    }

    next_drive_health_check_ = now + 20ms;
  }

  double drive_feedback(std::size_t index)
  {
    return drive_feedback_sample(index).output_rpm;
  }

  IswvDrive::Feedback drive_feedback_sample(std::size_t index)
  {
    try {
      return drive_wheels_[index]->actual_feedback_pdo();
    } catch (const std::exception & error) {
      throw std::runtime_error(
              std::string(kNames[index]) + " CAN" +
              std::to_string(config_.drive_can_buses[index]) + ": " + error.what());
    }
  }

  void stop_drives() noexcept
  {
    if (!steering_) {return;}
    for (std::size_t index = 0; index < kWheelCount; ++index) {
      const auto axis = steering_->at(static_cast<iswv::WheelPosition>(index));
      (void)axis->drive().quick_stop();
      (void)axis->drive().disable();
    }
  }

  void stop_drive_wheels() noexcept
  {
    for (auto & wheel : drive_wheels_) {
      if (wheel) {
        try {
          wheel->stop();
        } catch (...) {
        }
      }
    }
  }

  SteeringConfigFile config_;
  Hardware hardware_;
  State state_{State::uninitialized};
  bool initialized_{false};
  bool power_attempted_{false};
  bool shutdown_{false};
  struct Command {double x{0.0}; double y{0.0}; double yaw{0.0};};
  Command command_;
  bool zero_command_active_{false};
  std::chrono::steady_clock::time_point next_zero_refresh_{};
  struct StopWindow
  {
    double min_rpm{std::numeric_limits<double>::infinity()};
    double max_rpm{-std::numeric_limits<double>::infinity()};
    std::int32_t min_position{std::numeric_limits<std::int32_t>::max()};
    std::int32_t max_position{std::numeric_limits<std::int32_t>::min()};
    int peak_current_raw{0};
    unsigned samples{0};
  };
  std::array<StopWindow, kWheelCount> stop_window_{};
  std::chrono::steady_clock::time_point stop_report_start_{};
  std::chrono::steady_clock::time_point next_stop_report_{};
  std::string stop_diagnostic_;
  std::chrono::steady_clock::time_point next_drive_health_check_{};
  std::array<SwerveModuleConfig, kWheelCount> modules_{};
  std::array<ModuleKinematics, kWheelCount> kinematics_{};
  double max_module_radius_m_{0.0};
  double wheel_diameter_m_;
  double drive_max_output_rpm_;
  std::array<double, kWheelCount> last_angles_{};
  std::optional<std::array<WheelTarget, kWheelCount>> previous_targets_;
  bool alignment_paused_{false};
  std::chrono::steady_clock::time_point last_control_time_{};
  double smoothed_yaw_{0.0};
  std::array<std::int32_t, 4> safe_lower_inc_{};
  std::array<std::int32_t, 4> safe_upper_inc_{};
  std::shared_ptr<SteeringFeedback> steering_feedback_ = std::make_shared<SteeringFeedback>();
  std::array<iswv::Subscription, 8> steering_subscriptions_;
  std::array<double, kWheelCount> last_drive_rpm_{};
  std::array<std::int32_t, kWheelCount> zero_offsets_{};
  std::array<std::shared_ptr<iswv::ICanTransport>, 3> transports_{};
  std::array<std::unique_ptr<iswv::canopen::CanopenMaster>, 3> masters_{};
  std::unique_ptr<SteeringAxes> steering_;
  std::array<std::unique_ptr<IswvDrive>, kWheelCount> drive_wheels_{};
};

ChassisController::ChassisController(SteeringConfigFile config)
: ChassisController(std::move(config), Hardware{}) {}
ChassisController::ChassisController(SteeringConfigFile config, Hardware hardware)
: impl_(std::make_unique<Impl>(std::move(config), std::move(hardware))) {}
ChassisController::~ChassisController() = default;
void ChassisController::initialize(const std::function<bool()> & keep_running)
{impl_->initialize(keep_running);}
void ChassisController::calibrate(const std::function<bool()> & keep_running)
{impl_->calibrate(keep_running);}
void ChassisController::forward(double rpm) {impl_->forward(rpm);}
void ChassisController::set_velocity(double x, double y, double yaw)
{impl_->set_velocity(x, y, yaw);}
void ChassisController::update() {impl_->update();}
void ChassisController::stop() {impl_->stop();}
std::string ChassisController::take_stop_diagnostic()
{return std::exchange(impl_->stop_diagnostic_, {});}
ChassisController::State ChassisController::state() const noexcept {return impl_->state_;}
const SteeringConfigFile & ChassisController::config() const noexcept {return impl_->config_;}


namespace
{

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
