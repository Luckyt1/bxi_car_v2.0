#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace chassis
{

// 机械转角范围与单步限速，供舵轮解算和控制器共同使用。
struct SteeringRange
{
  double lower{-3.14159265358979323846};
  double upper{3.14159265358979323846};
};

inline double slew_towards(double current, double target, double max_step)
{
  return current + std::clamp(target - current, -max_step, max_step);
}

struct ModuleKinematics
{
  double x_m{0.0};
  double y_m{0.0};
  SteeringRange limits{};
  double heading_offset_rad{0.0};
  double max_speed_m_s{1.0};
};

struct SwerveOptions
{
  double speed_epsilon_m_s{1e-4};
  double branch_hysteresis_rad{0.05};
};

struct WheelTarget
{
  double angle_rad{0.0};
  double speed_m_s{0.0};
};

enum SolveStatus
{
  ok,
  invalid_configuration,
  invalid_command,
  invalid_feedback,
  feedback_out_of_range,
  no_candidate
};

struct SwerveSolution
{
  SolveStatus status{ok};
  std::size_t wheel{0};
  std::string reason{};
  std::array<WheelTarget, 4> targets{};
  double speed_scale{1.0};

  explicit operator bool() const {return status == ok;}
};

namespace bounded_swerve_detail
{
constexpr double pi = 3.14159265358979323846;
constexpr double max_reasonable_range_rad = 10000.0 * pi;
constexpr int max_candidate_count = 20000;

inline bool finite(double value)
{
  return std::isfinite(value);
}

inline bool inside_range(double angle, const SteeringRange & range)
{
  return finite(angle) && angle >= range.lower && angle <= range.upper;
}

inline double boundary_tolerance(double angle, const SteeringRange & range)
{
  const double scale =
    std::max({1.0, std::abs(angle), std::abs(range.lower), std::abs(range.upper)});
  return 64.0 * std::numeric_limits<double>::epsilon() * scale;
}

inline bool inside_range(double angle, const SteeringRange & range, double tolerance)
{
  return finite(angle) && angle >= range.lower - tolerance && angle <= range.upper + tolerance;
}

inline double snap_to_range(double angle, const SteeringRange & range, double tolerance)
{
  if (angle < range.lower && range.lower - angle <= tolerance) {
    return range.lower;
  }
  if (angle > range.upper && angle - range.upper <= tolerance) {
    return range.upper;
  }
  return angle;
}

inline SwerveSolution failure(SolveStatus status, std::size_t wheel, std::string reason)
{
  SwerveSolution result;
  result.status = status;
  result.wheel = wheel;
  result.reason = std::move(reason);
  result.speed_scale = 0.0;
  return result;
}

inline bool valid_range(const SteeringRange & range)
{
  return finite(range.lower) && finite(range.upper) && range.lower <= range.upper &&
         std::abs(range.lower) <= max_reasonable_range_rad &&
         std::abs(range.upper) <= max_reasonable_range_rad &&
         range.upper - range.lower <= max_reasonable_range_rad;
}

inline SwerveSolution validate_inputs(
  double vx, double vy, double omega, const std::array<ModuleKinematics, 4> & modules,
  const std::array<double, 4> & actual, const SwerveOptions & options)
{
  if (!finite(vx) || !finite(vy) || !finite(omega)) {
    return failure(invalid_command, 0, "command contains non-finite vx, vy, or omega");
  }
  if (!finite(options.speed_epsilon_m_s) || !finite(options.branch_hysteresis_rad) ||
    options.speed_epsilon_m_s < 0.0 || options.branch_hysteresis_rad < 0.0)
  {
    return failure(invalid_configuration, 0, "swerve options are invalid");
  }
  for (std::size_t index = 0; index < modules.size(); ++index) {
    const auto & module = modules[index];
    if (!finite(module.x_m) || !finite(module.y_m) || !finite(module.heading_offset_rad) ||
      !finite(module.max_speed_m_s) || module.max_speed_m_s < 0.0 ||
      std::abs(module.heading_offset_rad) > max_reasonable_range_rad ||
      !valid_range(module.limits))
    {
      return failure(invalid_configuration, index, "module configuration is invalid");
    }
    if (!finite(actual[index])) {
      return failure(invalid_feedback, index, "steering feedback is non-finite");
    }
    if (!inside_range(actual[index], module.limits)) {
      return failure(feedback_out_of_range, index, "steering feedback is outside limits");
    }
  }
  return {};
}

inline std::optional<WheelTarget> select_candidate(
  double physical_heading, double speed_m_s, double actual_angle, const ModuleKinematics & module,
  double hysteresis_rad, std::optional<WheelTarget> previous)
{
  const double base_joint = physical_heading - module.heading_offset_rad;
  const auto & limits = module.limits;
  const double tolerance = boundary_tolerance(base_joint, limits);
  const double n_min_value = std::ceil((limits.lower - base_joint - tolerance) / pi);
  const double n_max_value = std::floor((limits.upper - base_joint + tolerance) / pi);
  if (!finite(n_min_value) || !finite(n_max_value)) {
    return std::nullopt;
  }

  const int n_min = static_cast<int>(n_min_value);
  const int n_max = static_cast<int>(n_max_value);
  if (n_min > n_max || n_max - n_min > max_candidate_count) {
    return std::nullopt;
  }

  WheelTarget best{};
  double best_distance = std::numeric_limits<double>::infinity();
  std::optional<WheelTarget> continuity_candidate;
  double continuity_distance = std::numeric_limits<double>::infinity();
  double continuity_to_previous = std::numeric_limits<double>::infinity();

  for (int n = n_min; n <= n_max; ++n) {
    const double candidate_angle = base_joint + static_cast<double>(n) * pi;
    if (!inside_range(candidate_angle, limits, tolerance)) {
      continue;
    }
    const WheelTarget candidate{
      snap_to_range(candidate_angle, limits, tolerance),
      (n % 2 == 0) ? speed_m_s : -speed_m_s};

    const double distance = std::abs(candidate.angle_rad - actual_angle);
    if (distance < best_distance ||
      (std::abs(distance - best_distance) <= 1e-12 && candidate.speed_m_s > best.speed_m_s))
    {
      best_distance = distance;
      best = candidate;
    }

    if (previous && std::isfinite(previous->angle_rad) && std::isfinite(previous->speed_m_s) &&
      inside_range(previous->angle_rad, limits) &&
      (std::abs(previous->speed_m_s) <= 1e-12 ||
      (candidate.speed_m_s >= 0.0) == (previous->speed_m_s >= 0.0)))
    {
      const double to_previous = std::abs(candidate.angle_rad - previous->angle_rad);
      if (to_previous < continuity_to_previous) {
        continuity_to_previous = to_previous;
        continuity_candidate = candidate;
        continuity_distance = distance;
      }
    }
  }

  if (!std::isfinite(best_distance)) {
    return std::nullopt;
  }
  if (continuity_candidate && continuity_distance <= best_distance + hysteresis_rad) {
    return continuity_candidate;
  }
  return best;
}
}  // namespace bounded_swerve_detail

inline SwerveSolution solve_bounded_swerve(
  double vx, double vy, double omega, const std::array<ModuleKinematics, 4> & modules,
  const std::array<double, 4> & actual, const SwerveOptions & options = {},
  std::optional<std::array<WheelTarget, 4>> previous = std::nullopt)
{
  auto validation = bounded_swerve_detail::validate_inputs(vx, vy, omega, modules, actual, options);
  if (!validation) {
    return validation;
  }

  SwerveSolution solution;
  double speed_scale = 1.0;

  for (std::size_t index = 0; index < solution.targets.size(); ++index) {
    const auto & module = modules[index];
    const double wheel_vx = vx - omega * module.y_m;
    const double wheel_vy = vy + omega * module.x_m;
    const double speed = std::hypot(wheel_vx, wheel_vy);
    if (!bounded_swerve_detail::finite(speed)) {
      return bounded_swerve_detail::failure(
        invalid_command, index, "wheel velocity calculation produced a non-finite speed");
    }

    if (speed <= options.speed_epsilon_m_s) {
      const auto held = previous && bounded_swerve_detail::inside_range(
        (*previous)[index].angle_rad, module.limits) &&
        std::isfinite((*previous)[index].angle_rad) ?
        (*previous)[index].angle_rad : actual[index];
      solution.targets[index] = {held, 0.0};
      continue;
    }

    const double physical_heading = std::atan2(wheel_vy, wheel_vx);
    auto selected = bounded_swerve_detail::select_candidate(
      physical_heading, speed, actual[index], module, options.branch_hysteresis_rad,
      previous ? std::optional<WheelTarget>{(*previous)[index]} : std::nullopt);
    if (!selected) {
      return bounded_swerve_detail::failure(
        no_candidate, index,
        "no steering candidate inside limits");
    }

    solution.targets[index] = *selected;
    if (module.max_speed_m_s == 0.0) {
      if (std::abs(selected->speed_m_s) > 0.0) {
        return bounded_swerve_detail::failure(
          invalid_configuration, index,
          "nonzero wheel speed requested for a zero max speed module");
      }
    } else {
      speed_scale = std::min(speed_scale, module.max_speed_m_s / std::abs(selected->speed_m_s));
    }
  }

  if (speed_scale < 1.0) {
    solution.speed_scale = speed_scale;
    for (auto & target : solution.targets) {
      target.speed_m_s *= solution.speed_scale;
    }
  }

  return solution;
}

}  // namespace chassis
