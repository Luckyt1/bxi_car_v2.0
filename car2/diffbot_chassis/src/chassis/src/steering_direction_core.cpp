#include "chassis/steering_direction.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <thread>

namespace chassis
{
namespace
{
template<typename T>
T checked(iswv::Result<T> result)
{
  if (!result) {throw std::runtime_error(result.error().message);}
  return result.take_value();
}
void checked(iswv::Result<void> result)
{
  if (!result) {throw std::runtime_error(result.error().message);}
}
}  // namespace

double test_positive_steering_direction(
  iswv::SteeringLayout & steering, const SteeringConfigFile & config,
  const SteeringCalibrationResult & limits, std::size_t index,
  const std::function<bool()> & keep_running)
{
  using namespace std::chrono_literals;
  if (index >= 4) {throw std::invalid_argument("invalid steering wheel index");}
  validate_chassis_config(config);
  const auto running = [&] {
      if (!keep_running()) {throw std::runtime_error("direction confirmation cancelled");}
    };
  const auto axis = steering.at(static_cast<iswv::WheelPosition>(index));
  try {
    running();
    const double counts_per_deg = config.motor.encoder_resolution * config.motor.gear_ratio / 360.0;
    const auto initial = checked(axis->node()->read(iswv::objects::actual_position));
    const auto delta = static_cast<std::int64_t>(std::floor(50.0 * counts_per_deg));
    const auto target = static_cast<std::int64_t>(initial) + delta;
    const double margin = std::ceil(
      config.steering_limit_margin_rad * counts_per_deg *
      180.0 / std::acos(-1.0));
    const auto centered_error = static_cast<std::int64_t>(initial) - limits.midpoints[index];
    if (delta <= 0 || std::abs(centered_error) > std::max(1.0, counts_per_deg * 0.5) ||
      initial < limits.negative_limits[index] + margin ||
      target > limits.positive_limits[index] - margin ||
      std::abs(target - limits.midpoints[index]) / counts_per_deg >
      config.max_steering_angle_rad * 180.0 / std::acos(-1.0))
    {
      throw std::runtime_error("fresh midpoint/limit check failed before positive jog");
    }
    checked(axis->drive().transition_to(iswv::cia402::DriveState::switch_on_disabled));
    checked(axis->command_motor_velocity_rpm(0));
    checked(axis->configure_position_mode(2.0 * config.motor.gear_ratio, 1.0, 1.0));
    if (checked(axis->node()->read(iswv::objects::mode)) !=
      static_cast<std::int8_t>(iswv::cia402::OperationMode::profile_position))
    {
      throw std::runtime_error("position mode mismatch");
    }
    checked(axis->node()->write(iswv::objects::target_position, initial));
    running();
    checked(axis->drive().enable());
    running();
    std::cout << "DIRECTION_TEST wheel=" << index + 1
              << " node=" << static_cast<unsigned>(config.motor.node_ids[index])
              << " inverted=" << config.motor.inverted[index]
              << " command_deg=" << delta / counts_per_deg << " start_inc=" << initial
              << " target_inc=" << target << std::endl;
    checked(axis->command_position_pdo(static_cast<std::int32_t>(target), false));
    const auto start = std::chrono::steady_clock::now();
    const auto deadline = start + 10s;
    auto next_report = start;
    while (true) {
      running();
      const auto status = checked(axis->drive().read_status());
      const auto error1 = checked(axis->node()->read(iswv::objects::error_status));
      const auto error2 = checked(axis->node()->read(iswv::objects::error_status_2));
      const auto position = checked(axis->node()->read(iswv::objects::actual_position));
      const auto current = checked(axis->node()->read(iswv::objects::actual_current));
      const double moved = (static_cast<std::int64_t>(position) - initial) / counts_per_deg;
      const double torque = std::abs(static_cast<double>(current)) *
        (config.motor_peak_current_a / 1.414) / 2048.0 *
        config.motor_torque_constant_nm_per_a * config.motor.gear_ratio;
      if (iswv::cia402::decode_state(status) != iswv::cia402::DriveState::operation_enabled ||
        error1 || error2 || torque >= std::min(1.0, config.calibration_torque_limit_nm) ||
        moved < -0.5 || moved > 51.0 ||
        position < limits.negative_limits[index] + margin ||
        position > limits.positive_limits[index] - margin)
      {
        throw std::runtime_error(
                "positive jog protection: fault/torque/position; moved_deg=" +
                std::to_string(moved));
      }
      if (std::abs(static_cast<double>(position) - target) <=
        std::max(1.0, counts_per_deg * 0.15))
      {
        std::cout << "DIRECTION_RESULT wheel=" << index + 1 << " encoder_delta_deg=" << moved
                  << std::endl;
        return moved;  // Hold this target while user records the observation.
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_report) {
        std::cout << "DIRECTION_PROGRESS wheel=" << index + 1
                  << " position_inc=" << position << " target_inc=" << target
                  << " moved_deg=" << moved << " torque_nm=" << torque << std::endl;
        next_report = now + 1s;
      }
      if (now >= deadline) {
        std::ostringstream message;
        message << "positive jog timed out: wheel=" << index + 1
                << " node=" << static_cast<unsigned>(config.motor.node_ids[index])
                << " start_inc=" << initial << " position_inc=" << position
                << " target_inc=" << target << " moved_deg=" << moved
                << " remaining_deg=" << (target - position) / counts_per_deg
                << " torque_nm=" << torque << " status=0x" << std::hex << status
                << " error1=0x" << error1 << " error2=0x" << error2 << std::dec;
        throw std::runtime_error(message.str());
      }
      std::this_thread::sleep_for(20ms);
    }
  } catch (...) {
    (void)axis->drive().quick_stop();
    (void)axis->drive().disable();
    throw;
  }
}
}  // namespace chassis
