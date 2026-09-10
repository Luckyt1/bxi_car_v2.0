#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

namespace chassis
{

struct SwerveCommand
{
  double steering_rad{0.0};
  double drive_rpm{0.0};
};

struct SteeringRange
{
  double lower{-3.14159265358979323846};
  double upper{3.14159265358979323846};
};

inline double slew_towards(double current, double target, double max_step)
{
  return current + std::clamp(target - current, -max_step, max_step);
}

inline double wrap_to_pi(double angle)
{
  constexpr double pi = 3.14159265358979323846;
  while (angle > pi) {angle -= 2.0 * pi;}
  while (angle < -pi) {angle += 2.0 * pi;}
  return angle;
}

inline std::array<SwerveCommand, 4> swerve_commands(
  double linear_x, double linear_y, double angular_z,
  double wheel_base_m, double track_width_m, double wheel_diameter_m,
  double max_output_rpm, const std::array<double, 4> & current_angles,
  double steering_forward_offset_rad = 0.0,
  const std::array<SteeringRange, 4> & ranges = {},
  std::optional<std::array<double, 4>> previous_angles = std::nullopt)
{
  constexpr double pi = 3.14159265358979323846;
  const double half_base = 0.5 * wheel_base_m;
  const double half_track = 0.5 * track_width_m;
  const std::array<double, 4> x{half_base, -half_base, -half_base, half_base};
  const std::array<double, 4> y{half_track, half_track, -half_track, -half_track};
  std::array<SwerveCommand, 4> commands{};
  double largest_rpm = 0.0;

  for (std::size_t index = 0; index < commands.size(); ++index) {
    const double vx = linear_x - angular_z * y[index];
    const double vy = linear_y + angular_z * x[index];
    const double speed_m_s = std::hypot(vx, vy);
    if (speed_m_s < 1e-9) {
      commands[index].steering_rad = current_angles[index];
      continue;
    }

    const double base = wrap_to_pi(std::atan2(vy, vx) + steering_forward_offset_rad);
    double angle = 0.0;
    double direction = 1.0;
    double best_cost = std::numeric_limits<double>::infinity();
    // These are absolute mechanical angles: never wrap a path across a hard stop.
    for (const int half_turns : {0, -1, 1, -2, 2}) {
      const double candidate = base + half_turns * pi;
      if (candidate < ranges[index].lower || candidate > ranges[index].upper) {continue;}
      double cost = std::abs(candidate - current_angles[index]);
      if (previous_angles && std::abs(candidate - (*previous_angles)[index]) > pi / 2.0) {
        cost += 0.1;  // Hysteresis prevents +/-90 degree branch chatter.
      }
      if (cost < best_cost) {
        best_cost = cost;
        angle = candidate;
        direction = half_turns % 2 == 0 ? 1.0 : -1.0;
      }
    }
    if (!std::isfinite(best_cost)) {
      throw std::runtime_error(
              "no reachable steering angle inside calibrated limits for wheel " +
              std::to_string(index));
    }
    commands[index].steering_rad = angle;
    commands[index].drive_rpm = direction * speed_m_s * 60.0 / (pi * wheel_diameter_m);
    largest_rpm = std::max(largest_rpm, std::abs(commands[index].drive_rpm));
  }

  if (largest_rpm > max_output_rpm) {
    const double scale = max_output_rpm / largest_rpm;
    for (auto & command : commands) {
      command.drive_rpm *= scale;
    }
  }
  return commands;
}

}  // namespace chassis
