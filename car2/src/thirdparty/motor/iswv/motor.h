#pragma once

#include "motor/iswv/communication.h"
#include "iswv/cia402/drive.hpp"
#include "iswv/iswv/axis.hpp"

#include <cstdint>
#include <functional>
#include <memory>

// iswv::Axis exposes position/velocity/torque, homing and parameter operations;
// axis.drive() exposes CiA402 enable/disable/mode transitions, and axis.node()
// exposes typed parameter reads/writes. Each operation retains its declared units.
namespace chassis
{

// iSWV 行进电机 PV 模式；速度为减速器输出轴 RPM，加减速度为输出轴 RPM/s。
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
  // Runs synchronously after clearing velocity and confirming disabled state,
  // before PDO/mode configuration and enable. A callback failure triggers stop().
  void enable_rpm(
    double acceleration_output_rpm_s, double deceleration_output_rpm_s,
    const std::function<void(iswv::Axis &)> & configure_disabled);
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
