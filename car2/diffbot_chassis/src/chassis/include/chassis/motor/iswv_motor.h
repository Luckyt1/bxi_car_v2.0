#pragma once

#include "iswv/canopen/master.hpp"
#include "iswv/iswv/axis.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>

namespace chassis
{

struct SteeringConfigFile;

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

}  // namespace chassis
