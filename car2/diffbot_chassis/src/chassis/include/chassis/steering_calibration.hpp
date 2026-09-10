#pragma once

#include "chassis/steering_config.hpp"
#include "iswv/iswv.hpp"

#include <array>
#include <cstdint>
#include <functional>

namespace chassis
{

struct SteeringCalibrationResult
{
  std::array<std::int32_t, 4> midpoints{};
  std::array<std::int32_t, 4> negative_limits{};
  std::array<std::int32_t, 4> positive_limits{};
};

SteeringCalibrationResult calibrate_steering_with_limits(
  iswv::SteeringLayout & steering,
  const SteeringConfigFile & config,
  const std::function<bool()> & keep_running);

std::array<std::int32_t, 4> calibrate_steering(
  iswv::SteeringLayout & steering,
  const SteeringConfigFile & config,
  const std::function<bool()> & keep_running);

}  // namespace chassis
