#include "chassis/steering_config.hpp"

#include <limits>
#include <stdexcept>

int main()
{
  chassis::SteeringConfigFile config;
  chassis::validate_chassis_config(config);
  auto rejects = [](const chassis::SteeringConfigFile & value) {
      try {
        chassis::validate_chassis_config(value);
      } catch (const std::exception &) {
        return;
      }
      throw std::runtime_error("invalid chassis configuration accepted");
    };
  auto bad = config;
  bad.drive_can_buses[0] = 0;
  rejects(bad);
  bad = config;
  bad.drive_node_ids[1] = bad.drive_node_ids[0];
  rejects(bad);
  bad = config;
  bad.motor.node_ids[1] = bad.motor.node_ids[0];
  rejects(bad);
  bad = config;
  bad.drive_gear_ratio = 0;
  rejects(bad);
  bad = config;
  bad.drive_encoder_resolution = 0;
  rejects(bad);
  bad = config;
  bad.command_timeout_s = std::numeric_limits<double>::quiet_NaN();
  rejects(bad);
  bad = config;
  bad.calibration_stall_time_s = -1;
  rejects(bad);
  bad = config;
  bad.can_bus = 1;
  rejects(bad);
}
