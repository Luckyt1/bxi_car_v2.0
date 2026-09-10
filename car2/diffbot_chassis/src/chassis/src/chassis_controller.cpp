#include "chassis/chassis_controller.hpp"
#include "chassis/bxi_pci_transport.hpp"
#include "chassis/iswv_drive.hpp"
#include "chassis/steering_calibration.hpp"
#include "chassis/swerve_kinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>
#include <mutex>
#include <iostream>
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
    wheel_base_m_ = config_.wheel_base_m;
    track_width_m_ = config_.track_width_m;
    max_steering_angle_rad_ = config_.max_steering_angle_rad;
    steering_forward_offset_rad_ = config_.steering_forward_offset_rad;
    wheel_diameter_m_ = config_.motor.wheel_diameter_m;
    drive_max_output_rpm_ = config_.drive_max_output_rpm;
    drive_steering_tolerance_rad_ = config_.drive_steering_tolerance_rad;
    drive_inverted_ = config_.drive_inverted;
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
      steering_ = std::make_unique<iswv::SteeringLayout>(*masters_[0], config_.motor);
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
        axis_config.encoder_resolution = config_.drive_encoder_resolution;
        axis_config.gear_ratio = config_.drive_gear_ratio;
        axis_config.wheel_diameter_m = wheel_diameter_m_;
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
      const double counts_per_rad = config_.motor.encoder_resolution *
        config_.motor.gear_ratio / (2.0 * kPi);
      const auto margin = static_cast<std::int64_t>(
        std::ceil(config_.steering_limit_margin_rad * counts_per_rad));
      for (std::size_t i = 0; i < kWheelCount; ++i) {
        const auto lo = static_cast<std::int64_t>(calibration.negative_limits[i]) + margin;
        const auto hi = static_cast<std::int64_t>(calibration.positive_limits[i]) - margin;
        if (lo >= zero_offsets_[i] || hi <= zero_offsets_[i]) {
          throw std::runtime_error(
                  std::string(
                    kNames[i]) + " calibrated travel too small for margin");
        }
        safe_lower_inc_[i] = static_cast<std::int32_t>(lo);
        safe_upper_inc_[i] = static_cast<std::int32_t>(hi);
        steering_ranges_[i] = {
          std::max(-max_steering_angle_rad_, (lo - zero_offsets_[i]) / counts_per_rad),
          std::min(max_steering_angle_rad_, (hi - zero_offsets_[i]) / counts_per_rad)};
        std::cout << kNames[i] << " runtime steering range=[" << steering_ranges_[i].lower
                  << ", " << steering_ranges_[i].upper << "] rad, safe encoder=["
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
      for (auto & wheel : drive_wheels_) {
        if (!keep_running()) {throw std::runtime_error("chassis initialization cancelled");}
        wheel->enable_rpm(config_.drive_acceleration_rpm_s, config_.drive_deceleration_rpm_s);
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
      steering_ready_ = true;
      last_control_time_ = std::chrono::steady_clock::now();
      steering_progress_time_.fill(last_control_time_);
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
        !std::isfinite(std::hypot(x, y) + std::abs(yaw) * std::max(wheel_base_m_, track_width_m_)))
      {
        throw std::invalid_argument("chassis velocity must be finite and representable");
      }
      command_ = {x, y, yaw};
      last_command_time_ = std::chrono::steady_clock::now();
      state_ = State::commanded;
    } catch (...) {fail(); throw;}
  }

  void update()
  {
    require_ready();
    try {
      if (state_ == State::commanded &&
        std::chrono::duration<double>(
          std::chrono::steady_clock::now() -
          last_command_time_).count() >
        config_.command_timeout_s)
      {
        command_ = {};
        state_ = State::stopped;
      }
      control_once();
    } catch (...) {fail(); throw;}
  }

  void stop()
  {
    require_ready();
    try {
      command_ = {};
      stop_drive_wheels_before_steering();
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
    auto increments = axis->steering_radians_to_inc(angle_rad);
    if (!increments) {return iswv::Result<void>::failure(increments.error());}
    const auto target = static_cast<std::int64_t>(increments.value()) + zero_offsets_[index];
    if (target < safe_lower_inc_[index] || target > safe_upper_inc_[index]) {
      return iswv::Result<void>::failure(
        iswv::ErrorCode::invalid_argument, "steering target outside calibrated safe encoder range");
    }
    return axis->command_position_pdo(static_cast<std::int32_t>(target), false);
  }

  void control_once()
  {
    check_steering_health();
    const auto actual = actual_steering_angles();
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::clamp(
      std::chrono::duration<double>(now - last_control_time_).count(), 0.0, 0.05);
    last_control_time_ = now;
    const double command_speed = std::hypot(command_.x, command_.y) +
      std::abs(command_.yaw) * std::max(wheel_base_m_, track_width_m_);
    if (command_speed < 1e-6) {
      smoothed_yaw_ = 0.0;
      realigning_ = false;
      steering_progress_time_.fill(now);
      command_zero_and_verify_stop();
      return;
    }
    zero_command_active_ = false;
    drive_wheels_stopped_ = false;
    check_drive_wheel_health();

    smoothed_yaw_ = slew_towards(
      smoothed_yaw_, command_.yaw,
      config_.command_yaw_acceleration_rad_s2 * dt);
    const auto targets = chassis::swerve_commands(
      command_.x, command_.y, smoothed_yaw_,
      wheel_base_m_, track_width_m_, wheel_diameter_m_, drive_max_output_rpm_, actual,
      steering_forward_offset_rad_, steering_ranges_, previous_goal_angles_);
    double largest_error = 0.0;
    for (std::size_t index = 0; index < kWheelCount; ++index) {
      largest_error =
        std::max(largest_error, std::abs(targets[index].steering_rad - actual[index]));
      previous_goal_angles_[index] = targets[index].steering_rad;
      if (std::abs(actual[index] - steering_progress_angle_[index]) > 0.01 ||
        std::abs(targets[index].steering_rad - actual[index]) <= drive_steering_tolerance_rad_)
      {
        steering_progress_angle_[index] = actual[index];
        steering_progress_time_[index] = now;
      } else if (now - steering_progress_time_[index] > 10s) {
        throw std::runtime_error(
                std::string(kNames[index]) +
                " steering made no progress for 10 s");
      }
    }
    if (!realigning_ && largest_error >= config_.drive_alignment_stop_rad) {
      realigning_ = true;
      alignment_stop_deadline_ = now + 2s;
      alignment_deadline_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(
          std::max(
            10.0,
            2.0 * largest_error / config_.steering_rate_limit_rad_s + 2.0)));
    }
    if (realigning_) {
      if (now >= alignment_deadline_) {
        throw std::runtime_error("large steering alignment timed out");
      }
      command_drive_wheels({0.0, 0.0, 0.0, 0.0});
      bool stopped = true;
      for (std::size_t i = 0; i < kWheelCount; ++i) {
        const double rpm = drive_feedback(i);
        stopped = stopped && std::abs(rpm) <= 0.05;
      }
      if (!stopped) {
        if (now >= alignment_stop_deadline_) {
          throw std::runtime_error("travel wheels failed to stop before large steering change");
        }
        return;  // Nonblocking: joystick cancellation remains responsive.
      }
      if (largest_error <= drive_steering_tolerance_rad_) {realigning_ = false;}
    }

    const double alignment_scale = realigning_ ? 0.0 : std::clamp(
      (config_.drive_alignment_stop_rad - largest_error) /
      (config_.drive_alignment_stop_rad - drive_steering_tolerance_rad_), 0.0, 1.0);
    std::array<double, kWheelCount> drive_rpm{};
    for (std::size_t index = 0; index < kWheelCount; ++index) {
      const double angle = slew_towards(
        last_angles_[index], targets[index].steering_rad,
        config_.steering_rate_limit_rad_s * dt);
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
      double requested = targets[index].drive_rpm * alignment_scale;
      if (drive_inverted_[index]) {requested = -requested;}
      const double previous = last_drive_rpm_[index];
      const double measured_rpm = drive_feedback(index);
      const bool reversing = requested * previous < 0.0 ||
        (requested * measured_rpm<0.0 && std::abs(measured_rpm)>0.05);
      const double goal = reversing ? 0.0 : requested;
      const double rate = std::abs(goal) > std::abs(previous) ?
        config_.drive_acceleration_rpm_s : config_.drive_deceleration_rpm_s;
      drive_rpm[index] = alignment_scale == 0.0 ? 0.0 : slew_towards(previous, goal, rate * dt);
    }
    command_drive_wheels(drive_rpm);
    check_drive_wheel_health();
  }

  bool steering_ready_for_drive()
  {
    check_steering_health();
    const auto actual = actual_steering_angles();
    for (std::size_t index = 0; index < kWheelCount; ++index) {
      if (std::abs(actual[index] - last_angles_[index]) > drive_steering_tolerance_rad_) {
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
    const double counts_per_rad = config_.motor.encoder_resolution *
      config_.motor.gear_ratio / (2.0 * kPi);
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      const auto position = iswv::canopen::decode_little_endian<std::int32_t>(
        frames[i][0].data.data() + 2, 4).value();
      if (position < safe_lower_inc_[i] || position > safe_upper_inc_[i]) {
        throw std::runtime_error(
                std::string(kNames[i]) + " actual steering outside safe limits: " +
                std::to_string(position) + " inc, allowed [" + std::to_string(safe_lower_inc_[i]) +
                ", " + std::to_string(safe_upper_inc_[i]) + "]");
      }
      result[i] = (static_cast<std::int64_t>(position) - zero_offsets_[i]) / counts_per_rad;
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

  void command_drive_wheels(const std::array<double, kWheelCount> & targets)
  {
    for (std::size_t index = 0; index < kWheelCount; ++index) {
      if (std::abs(targets[index] - last_drive_rpm_[index]) < 1e-4) {continue;}
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
        std::abs(actual_rpm) > drive_max_output_rpm_ * 1.25 + 0.1)
      {
        throw std::runtime_error(
                "drive wheel " + std::to_string(index) + " exceeded the RPM safety limit");
      }
    }
    next_drive_health_check_ = now + 20ms;
  }

  void command_zero_and_verify_stop()
  {
    const auto now = std::chrono::steady_clock::now();
    command_drive_wheels({0.0, 0.0, 0.0, 0.0});
    if (!zero_command_active_) {
      zero_command_active_ = true;
      drive_wheels_stopped_ = false;
      drive_stop_deadline_ = now + 2s;
      next_drive_health_check_ = now;
    }
    if (now < next_drive_health_check_) {return;}

    bool stopped = true;
    for (std::size_t index = 0; index < kWheelCount; ++index) {
      const double actual_rpm = drive_feedback(index);
      if (!std::isfinite(actual_rpm)) {
        throw std::runtime_error(
                "drive wheel " + std::to_string(index) + " returned non-finite RPM");
      }
      stopped = stopped && std::abs(actual_rpm) <= 0.05;
    }

    if (stopped) {
      drive_wheels_stopped_ = true;
      next_drive_health_check_ = now + 20ms;
      return;
    }
    if (drive_wheels_stopped_) {
      drive_wheels_stopped_ = false;
      drive_stop_deadline_ = now + 2s;
    }
    if (now >= drive_stop_deadline_) {
      throw std::runtime_error("drive wheels did not stop after the zero-speed command");
    }
    next_drive_health_check_ = now + 20ms;
  }

  double drive_feedback(std::size_t index)
  {
    constexpr std::array<const char *, 4> names{
      "front_left", "rear_left", "rear_right", "front_right"};
    try {
      return drive_wheels_[index]->actual_velocity_pdo_rpm();
    } catch (const std::exception & error) {
      throw std::runtime_error(
              std::string(names[index]) + " CAN" +
              std::to_string(config_.drive_can_buses[index]) + ": " + error.what());
    }
  }

  void stop_drive_wheels_before_steering()
  {
    command_drive_wheels({0.0, 0.0, 0.0, 0.0});
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (true) {
      bool stopped = true;
      for (std::size_t i = 0; i < kWheelCount; ++i) {
        const double rpm = drive_feedback(i);
        stopped = stopped && std::abs(rpm) <= 0.05;
      }
      if (stopped) {return;}
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("drive wheels did not stop before steering movement");
      }
      std::this_thread::sleep_for(20ms);
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
  std::chrono::steady_clock::time_point last_command_time_{};
  bool steering_ready_{false};
  bool zero_command_active_{false};
  bool drive_wheels_stopped_{false};
  std::chrono::steady_clock::time_point drive_stop_deadline_{};
  std::chrono::steady_clock::time_point next_drive_health_check_{};
  double wheel_base_m_;
  double track_width_m_;
  double wheel_diameter_m_;
  double max_steering_angle_rad_;
  double steering_forward_offset_rad_;
  double drive_max_output_rpm_;
  double drive_steering_tolerance_rad_;
  std::array<double, kWheelCount> last_angles_{};
  std::array<double, kWheelCount> previous_goal_angles_{};
  std::array<double, kWheelCount> steering_progress_angle_{};
  std::array<std::chrono::steady_clock::time_point, 4> steering_progress_time_{};
  std::chrono::steady_clock::time_point last_control_time_{};
  std::chrono::steady_clock::time_point alignment_stop_deadline_{};
  std::chrono::steady_clock::time_point alignment_deadline_{};
  bool realigning_{false};
  double smoothed_yaw_{0.0};
  std::array<SteeringRange, 4> steering_ranges_{};
  std::array<std::int32_t, 4> safe_lower_inc_{};
  std::array<std::int32_t, 4> safe_upper_inc_{};
  std::shared_ptr<SteeringFeedback> steering_feedback_ = std::make_shared<SteeringFeedback>();
  std::array<iswv::Subscription, 8> steering_subscriptions_;
  std::array<double, kWheelCount> last_drive_rpm_{};
  std::array<std::int32_t, kWheelCount> zero_offsets_{};
  std::array<bool, kWheelCount> drive_inverted_{};
  std::array<std::shared_ptr<iswv::ICanTransport>, 3> transports_{};
  std::array<std::unique_ptr<iswv::canopen::CanopenMaster>, 3> masters_{};
  std::unique_ptr<iswv::SteeringLayout> steering_;
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
ChassisController::State ChassisController::state() const noexcept {return impl_->state_;}
const SteeringConfigFile & ChassisController::config() const noexcept {return impl_->config_;}
}  // namespace chassis
