#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace chassis
{

struct RemoteKinematicsConfig
{
  double scale_linear_m_s{0.6};
  double scale_lateral_m_s{0.6};
  double scale_angular_rad_s{0.4};
};

struct RemoteMotionCommand
{
  double linear_x_m_s{0.0};
  double linear_y_m_s{0.0};
  double angular_z_rad_s{0.0};
};

inline double remote_smooth_axis(double value)
{
  constexpr double expo = 0.7;
  return (1.0 - expo) * value + expo * value * value * value;
}

inline double remote_apply_deadzone(double value, double deadzone)
{
  return std::abs(value) < deadzone ? 0.0 : value;
}

// Exact discrete first-order low-pass filter. The caller owns the previous
// output and applies the joystick deadzone separately.
inline double remote_low_pass_axis(
  double previous, double input, double dt_s, double time_constant_s)
{
  if (!std::isfinite(previous) || previous < -1.0 || previous > 1.0 ||
    !std::isfinite(input) || input < -1.0 || input > 1.0 ||
    !std::isfinite(dt_s) || dt_s < 0.0 ||
    !std::isfinite(time_constant_s) || time_constant_s < 0.0)
  {
    throw std::invalid_argument("invalid remote low-pass axis, interval or time constant");
  }
  if (time_constant_s == 0.0) {
    return input;
  }
  if (dt_s == 0.0) {
    return previous;
  }
  const double alpha = -std::expm1(-dt_s / time_constant_s);
  return std::clamp(previous + alpha * (input - previous), -1.0, 1.0);
}

inline void validate_remote_kinematics_config(const RemoteKinematicsConfig & config)
{
  if (!std::isfinite(config.scale_linear_m_s) || config.scale_linear_m_s <= 0.0 ||
    !std::isfinite(config.scale_lateral_m_s) || config.scale_lateral_m_s <= 0.0 ||
    !std::isfinite(config.scale_angular_rad_s) || config.scale_angular_rad_s <= 0.0)
  {
    throw std::invalid_argument("invalid remote kinematics scale");
  }
}

inline RemoteMotionCommand remote_motion_from_axes(
  double linear_axis, double lateral_axis, double angular_axis,
  const RemoteKinematicsConfig & config)
{
  validate_remote_kinematics_config(config);
  linear_axis = std::clamp(linear_axis, -1.0, 1.0);
  lateral_axis = std::clamp(lateral_axis, -1.0, 1.0);
  angular_axis = std::clamp(angular_axis, -1.0, 1.0);

  RemoteMotionCommand command;
  command.linear_x_m_s = remote_smooth_axis(linear_axis) * config.scale_linear_m_s;
  command.linear_y_m_s = remote_smooth_axis(lateral_axis) * config.scale_lateral_m_s;
  command.angular_z_rad_s = remote_smooth_axis(angular_axis) * config.scale_angular_rad_s;
  return command;
}

}  // namespace chassis
