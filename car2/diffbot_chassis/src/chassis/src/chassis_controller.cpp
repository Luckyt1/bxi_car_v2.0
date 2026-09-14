#include "chassis/chassis_controller.hpp"
#include "chassis/bxi_pci_transport.hpp"
#include "chassis/motor/iswv_motor.hpp"
#include "chassis/swerve_kinematics.hpp"

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
void require_success(const iswv::Result<T> & result, const char * operation)
{
  if (!result) {throw std::runtime_error(std::string(operation) + ": " + result.error().message);}
}
}  // namespace

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
}  // namespace chassis
