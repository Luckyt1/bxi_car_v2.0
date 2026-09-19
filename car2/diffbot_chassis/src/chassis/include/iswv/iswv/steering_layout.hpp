#pragma once

#include "iswv/iswv/axis.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace iswv
{

enum class WheelPosition : std::size_t
{
  front_left,
  rear_left,
  rear_right,
  front_right,
};

struct SteeringLayoutConfig
{
  std::array<std::uint8_t, 4> node_ids{1, 2, 3, 4};
  std::uint32_t encoder_resolution{65536};
  double gear_ratio{2500.0 / 277.0};
  double wheel_diameter_m{0.130};
  std::array<bool, 4> inverted{false, false, false, false};
  DefaultPdoOptions pdo{};
  double profile_velocity_rpm{200.0};
  double acceleration_rps2{100.0};
  double deceleration_rps2{100.0};

  static Result<SteeringLayoutConfig> from_file(const std::string & path);
};

class SteeringLayout
{
public:
  inline static constexpr std::array<std::uint8_t, 4> node_ids{1, 2, 3, 4};

  explicit SteeringLayout(canopen::CanopenMaster & can0)
  : SteeringLayout(can0, {}) {}

  SteeringLayout(canopen::CanopenMaster & can0, const SteeringLayoutConfig & configuration)
  {
    for (std::size_t index = 0; index < axes_.size(); ++index) {
      const auto node_id = configuration.node_ids[index];
      auto axis_configuration = AxisConfiguration::manual_steering(node_id);
      axis_configuration.encoder_resolution = configuration.encoder_resolution;
      axis_configuration.gear_ratio = configuration.gear_ratio;
      axis_configuration.wheel_diameter_m = configuration.wheel_diameter_m;
      axis_configuration.inverted = configuration.inverted[index];
      axes_[index] = std::make_shared<Axis>(can0.node(node_id), axis_configuration);
    }
  }

  std::shared_ptr<Axis> at(WheelPosition position) const
  {
    return axes_.at(static_cast<std::size_t>(position));
  }

  Result<void> configure(
    const SteeringLayoutConfig & configuration,
    canopen::SdoOptions sdo = {})
  {
    for (const auto & axis : axes_) {
      auto result = axis->configure_default_pdos(configuration.pdo, sdo);
      if (!result) {
        return result;
      }
      result = axis->configure_position_mode(
        configuration.profile_velocity_rpm,
        configuration.acceleration_rps2,
        configuration.deceleration_rps2,
        sdo);
      if (!result) {
        return result;
      }
    }
    return Result<void>::success();
  }

private:
  std::array<std::shared_ptr<Axis>, 4> axes_{};
};

}  // namespace iswv
