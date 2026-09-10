#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace chassis
{

class TorqueLimitDetector
{
public:
  TorqueLimitDetector(
    double contact_nm, double safety_nm, std::int32_t position_epsilon_inc = 0,
    std::chrono::duration<double> stall_time = std::chrono::duration<double>::zero())
  : contact_nm_(contact_nm), safety_nm_(safety_nm),
    position_epsilon_inc_(position_epsilon_inc), stall_time_(stall_time)
  {
    if (!std::isfinite(contact_nm) || !std::isfinite(safety_nm) ||
      contact_nm <= 0.0 || contact_nm >= safety_nm || position_epsilon_inc < 0 ||
      !std::isfinite(stall_time.count()) || stall_time.count() < 0.0)
    {
      throw std::invalid_argument("contact torque must be positive and below safety torque");
    }
  }

  bool update_contact(
    double output_torque_nm, bool detection_armed, std::int32_t position,
    std::chrono::steady_clock::time_point now)
  {
    if (!position_initialized_) {
      last_position_ = position;
      last_motion_time_ = now;
      position_initialized_ = true;
    }
    if (std::abs(static_cast<std::int64_t>(position) - last_position_) >
      position_epsilon_inc_)
    {
      last_motion_time_ = now;
      last_position_ = position;
    }
    const bool stalled = now - last_motion_time_ >= stall_time_;
    return update(output_torque_nm, detection_armed && stalled);
  }

  // Once armed, one sample is sufficient. Safety is checked even while disarmed.
  bool update(double output_torque_nm, bool detection_armed = true) const
  {
    if (!std::isfinite(output_torque_nm)) {
      throw std::runtime_error("估算输出力矩无效，停止校准（不记录限位）");
    }
    const double magnitude = std::abs(output_torque_nm);
    if (magnitude >= safety_nm_) {
      throw std::runtime_error(
              "估算输出力矩 " + std::to_string(magnitude) + " N·m 达到安全上限 " +
              std::to_string(safety_nm_) + " N·m，停止校准（不记录限位）");
    }
    return detection_armed && magnitude >= contact_nm_;
  }

private:
  double contact_nm_;
  double safety_nm_;
  std::int32_t position_epsilon_inc_;
  std::chrono::duration<double> stall_time_;
  bool position_initialized_{false};
  std::int32_t last_position_{0};
  std::chrono::steady_clock::time_point last_motion_time_{};
};

}  // namespace chassis
