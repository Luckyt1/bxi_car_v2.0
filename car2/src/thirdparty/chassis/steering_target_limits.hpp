#pragma once

#include "chassis/swerve_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace chassis
{

struct SteeringTargetLimits
{
  std::int32_t lower_inc;
  std::int32_t upper_inc;
  SteeringRange angles;
};

// Keep the original encoder safety boundaries; round the extra target inset inward.
inline SteeringTargetLimits inset_steering_targets(
  std::int32_t safe_lower, std::int32_t safe_upper, std::int32_t midpoint,
  int sign, double counts_per_rad, double margin_rad)
{
  if (safe_lower >= safe_upper || midpoint < safe_lower || midpoint > safe_upper ||
    (sign != -1 && sign != 1) || !std::isfinite(counts_per_rad) || counts_per_rad <= 0.0 ||
    !std::isfinite(margin_rad) || margin_rad < 0.0)
  {
    throw std::invalid_argument("invalid steering target margin geometry");
  }
  const double inset = std::ceil(margin_rad * counts_per_rad);
  const auto width = static_cast<std::int64_t>(safe_upper) - safe_lower;
  if (!std::isfinite(inset) || inset >= static_cast<double>(width) / 2.0) {
    throw std::invalid_argument("steering target margin leaves no usable calibrated range");
  }
  const auto lower = static_cast<std::int64_t>(safe_lower) + static_cast<std::int64_t>(inset);
  const auto upper = static_cast<std::int64_t>(safe_upper) - static_cast<std::int64_t>(inset);
  if (lower > midpoint || upper < midpoint) {
    throw std::invalid_argument("steering target margin excludes the calibrated midpoint");
  }
  const double a = sign * (lower - midpoint) / counts_per_rad;
  const double b = sign * (upper - midpoint) / counts_per_rad;
  return {static_cast<std::int32_t>(lower), static_cast<std::int32_t>(upper),
    {std::min(a, b), std::max(a, b)}};
}

}  // namespace chassis
