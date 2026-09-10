#pragma once

#include "iswv/cia402/drive.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace chassis
{

// PHU/RHU CANopen PV mode, yy.pdf V1.08 sections 4.7.2 and 6.2.
// Single caller; master and transport must outlive the motor. Errors throw.
class EyouMotor
{
public:
  EyouMotor(iswv::canopen::CanopenMaster & master, std::uint8_t node_id);
  ~EyouMotor();
  EyouMotor(const EyouMotor &) = delete;
  EyouMotor & operator=(const EyouMotor &) = delete;

  void enable(std::uint32_t acceleration, std::uint32_t deceleration);
  void set_velocity(std::int32_t pulses_per_second);
  std::int32_t actual_velocity();  // Also checks operation-enabled state.
  std::int32_t actual_position();  // Signed 0x6064 feedback, pulse units.
  void stop();  // Ramp to zero, then disable voltage; bounded wait.
  std::string diagnostic_report();  // Read-only; report each SDO failure independently.
  std::string information_report(bool * complete = nullptr);  // Read-only motor information.
  std::string electrical_report(bool * complete = nullptr);  // Optional telemetry, 50 ms/read.

  // RPM refers to the reducer OUTPUT shaft. Read scale before using RPM APIs.
  double configure_rpm_units();  // Returns pulse counts per output revolution.
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
