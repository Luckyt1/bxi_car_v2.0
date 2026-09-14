#pragma once

#include "chassis/steering_config.hpp"
#include "iswv/canopen/master.hpp"
#include "iswv/iswv.hpp"
#include "iswv/iswv/axis.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

namespace chassis
{

iswv::AxisConfiguration steering_axis_configuration(
  const SteeringConfigFile & config, std::size_t index);

// CAN0 四个舵向轴：只负责按配置创建和索引 Axis，校准细节放在 cpp。
class SteeringAxes
{
public:
  SteeringAxes(iswv::canopen::CanopenMaster & can0, const SteeringConfigFile & config);

  std::shared_ptr<iswv::Axis> at(iswv::WheelPosition position) const
  {
    return axes_.at(static_cast<std::size_t>(position));
  }

private:
  std::array<std::shared_ptr<iswv::Axis>, 4> axes_{};
};

// CAN1/2 的 iSWV 行进电机驱动，输出单位统一为减速器输出轴 RPM。
class IswvDrive
{
public:
  struct Feedback
  {
    double output_rpm;
    std::int32_t position_inc;
    std::int16_t current_raw;
  };

  IswvDrive(iswv::canopen::CanopenMaster & master, iswv::AxisConfiguration config);
  ~IswvDrive();

  IswvDrive(const IswvDrive &) = delete;
  IswvDrive & operator=(const IswvDrive &) = delete;
  IswvDrive(IswvDrive &&) = delete;
  IswvDrive & operator=(IswvDrive &&) = delete;

  void enable_rpm(double acceleration_output_rpm_s, double deceleration_output_rpm_s);
  void enable_rpm(
    double acceleration_output_rpm_s, double deceleration_output_rpm_s,
    SteeringConfigFile & tuning_config, std::size_t wheel_index);
  void set_velocity_rpm(double output_rpm);
  void set_velocity_pdo_rpm(double output_rpm);
  double actual_velocity_rpm();
  double actual_velocity_pdo_rpm();
  Feedback actual_feedback_pdo();
  void stop();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

struct SteeringCalibrationResult
{
  std::array<std::int32_t, 4> midpoints{};
  std::array<std::int32_t, 4> negative_limits{};
  std::array<std::int32_t, 4> positive_limits{};
};

SteeringCalibrationResult calibrate_steering_with_limits(
  iswv::SteeringLayout & steering,
  SteeringConfigFile & config,
  const std::function<bool()> & keep_running);

std::array<std::int32_t, 4> calibrate_steering(
  iswv::SteeringLayout & steering,
  SteeringConfigFile & config,
  const std::function<bool()> & keep_running);

SteeringCalibrationResult calibrate_steering_with_limits(
  SteeringAxes & steering,
  SteeringConfigFile & config,
  const std::function<bool()> & keep_running);

std::array<std::int32_t, 4> calibrate_steering(
  SteeringAxes & steering,
  SteeringConfigFile & config,
  const std::function<bool()> & keep_running);

}  // namespace chassis
