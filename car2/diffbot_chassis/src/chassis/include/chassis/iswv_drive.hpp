#pragma once

#include "iswv/canopen/master.hpp"
#include "iswv/iswv/axis.hpp"

#include <memory>

namespace chassis
{

class IswvDrive
{
public:
  IswvDrive(iswv::canopen::CanopenMaster & master, iswv::AxisConfiguration config);
  ~IswvDrive();

  IswvDrive(const IswvDrive &) = delete;
  IswvDrive & operator=(const IswvDrive &) = delete;
  IswvDrive(IswvDrive &&) = delete;
  IswvDrive & operator=(IswvDrive &&) = delete;

  void enable_rpm(
    double acceleration_output_rpm_s,
    double deceleration_output_rpm_s);
  void set_velocity_rpm(double output_rpm);
  void set_velocity_pdo_rpm(double output_rpm);
  double actual_velocity_rpm();
  double actual_velocity_pdo_rpm();
  void stop();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace chassis
