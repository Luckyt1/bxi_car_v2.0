#pragma once

#include "chassis/steering_config.hpp"
#include "chassis/steering_tuning_persistence.hpp"

#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace chassis
{

// These objects configure gain group 0. Do not select a gain group or save to
// driver flash here: both are separate operations from applying YAML overrides.
inline void apply_motor_tuning(
  iswv::Axis & axis, SteeringConfigFile & config, std::size_t index, bool travel)
{
  if (index >= 4) {throw std::invalid_argument("motor tuning wheel index must be 0..3");}
  validate_chassis_config(config);
  const std::string prefix = travel ? "drive_" : "steering_";
  const std::string label = (travel ?
    "drive tuning CAN" + std::to_string(config.drive_can_buses[index]) : "steering tuning") +
    " Node-ID " + std::to_string(axis.configuration().node_id);
  auto & velocity_kp = travel ? config.drive_velocity_loop_kp[index] :
    config.steering_velocity_loop_kp[index];
  auto & velocity_ki = travel ? config.drive_velocity_loop_ki[index] :
    config.steering_velocity_loop_ki[index];
  auto & feedback_filter = travel ? config.drive_velocity_feedback_filter[index] :
    config.steering_velocity_feedback_filter[index];
  auto & position_kp = travel ? config.drive_position_loop_kp[index] :
    config.steering_position_loop_kp[index];
  auto & smoothing_filter = travel ? config.drive_position_smoothing_filter[index] :
    config.steering_position_smoothing_filter[index];
  auto node = axis.node();
  if (velocity_kp || velocity_ki || feedback_filter || position_kp || smoothing_filter) {
    const auto status = axis.drive().read_status();
    if (!status) {
      throw std::runtime_error(label + " read 0x6041:00 failed: " + status.error().message);
    }
    if (iswv::cia402::decode_state(status.value()) !=
      iswv::cia402::DriveState::switch_on_disabled)
    {
      throw std::runtime_error(label + " requires switch_on_disabled before writing parameters");
    }
  }

  const auto apply = [&](auto object, std::optional<int> & desired, const std::string & name,
      int minimum, int maximum) {
      std::ostringstream address;
      address << "0x" << std::hex << object.address.index << ":"
              << static_cast<unsigned>(object.address.subindex);
      const auto context = label + " " + name + " " + address.str();
      const auto before = node->read(object);
      if (!before) {
        if (!desired) {
          std::cerr << context << " read unavailable: " << before.error().message <<
            "; null override, unchanged\n";
          return;
        }
        throw std::runtime_error(context + " read failed: " + before.error().message);
      }
      if (!desired) {
        std::cout << context << " raw " << static_cast<int>(before.value()) <<
          " (null override, unchanged)\n";
        const int raw = static_cast<int>(before.value());
        if (raw >= minimum && raw <= maximum) {
          desired = raw;
        } else {
          std::cerr << context << " outside supported configuration range; keeping null\n";
        }
        return;
      }
      if (static_cast<int>(before.value()) != *desired) {
        const auto written = node->write(object, *desired);
        if (!written) {
          throw std::runtime_error(context + " write failed: " + written.error().message);
        }
        const auto after = node->read(object);
        if (!after) {
          throw std::runtime_error(context + " readback failed: " + after.error().message);
        }
        if (static_cast<int>(after.value()) != *desired) {
          throw std::runtime_error(
                  context + " readback mismatch: expected=" + std::to_string(*desired) +
                  " actual=" + std::to_string(static_cast<int>(after.value())));
        }
      }
      std::cout << context << " raw " << static_cast<int>(before.value())
                << " -> " << *desired << " verified\n";
    };
  apply(iswv::objects::velocity_loop_kp, velocity_kp, prefix + "velocity_loop_kp", 1, 32767);
  apply(iswv::objects::velocity_loop_ki, velocity_ki, prefix + "velocity_loop_ki", 0, 1023);
  apply(
    iswv::objects::velocity_feedback_filter, feedback_filter,
    prefix + "velocity_feedback_filter", 0, 45);
  apply(iswv::objects::position_loop_kp, position_kp, prefix + "position_loop_kp", 0, 32767);
  apply(
    iswv::objects::position_smoothing_filter, smoothing_filter,
    prefix + "position_smoothing_filter", 1, 255);
  persist_steering_tuning(config.source_path, config);
}

inline void apply_steering_tuning(
  iswv::Axis & axis, SteeringConfigFile & config, std::size_t index)
{
  apply_motor_tuning(axis, config, index, false);
}

inline void apply_drive_tuning(
  iswv::Axis & axis, SteeringConfigFile & config, std::size_t index)
{
  apply_motor_tuning(axis, config, index, true);
}

}  // namespace chassis
