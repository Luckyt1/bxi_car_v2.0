#pragma once

#include "chassis/steering_calibration.hpp"

namespace chassis
{
// Uses this power session's measured limits, not persisted encoder coordinates.
double test_positive_steering_direction(
  iswv::SteeringLayout & steering, const SteeringConfigFile & config,
  const SteeringCalibrationResult & limits, std::size_t index,
  const std::function<bool()> & keep_running);
}  // namespace chassis
