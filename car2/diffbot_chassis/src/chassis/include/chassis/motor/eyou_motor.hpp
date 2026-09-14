#pragma once

#include "iswv/cia402/drive.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace chassis
{

// 意优 PHU/RHU：CANopen PV 模式驱动接口，只保留主流程需要的运动与诊断能力。
class EyouMotor
{
public:
  EyouMotor(iswv::canopen::CanopenMaster & master, std::uint8_t node_id);
  ~EyouMotor();
  EyouMotor(const EyouMotor &) = delete;
  EyouMotor & operator=(const EyouMotor &) = delete;

  void enable(std::uint32_t acceleration, std::uint32_t deceleration);
  void set_velocity(std::int32_t pulses_per_second);
  std::int32_t actual_velocity();
  std::int32_t actual_position();
  void stop();
  std::string diagnostic_report();

  // RPM 指减速器输出轴；使用 RPM API 前必须先读取电机的编码器/减速比标尺。
  double configure_rpm_units();
  std::int32_t velocity_from_rpm(double rpm) const;
  void enable_rpm(double acceleration_rpm_per_s, double deceleration_rpm_per_s);
  void set_velocity_rpm(double rpm);
  double actual_velocity_rpm();

private:
  std::shared_ptr<iswv::canopen::CanopenNode> node_;
  iswv::cia402::Drive drive_;
  bool active_{false};
  bool enabled_{false};
  double pulses_per_output_revolution_{0};
};

}  // namespace chassis
