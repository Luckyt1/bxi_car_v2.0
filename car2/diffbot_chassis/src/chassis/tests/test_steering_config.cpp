#include "chassis/steering_config.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

int main(int argc, char ** argv)
{
  assert(argc == 2);
  const std::string source = argv[1];
  auto config = chassis::load_steering_config(source);
  assert(config.can_bus == 0);
  assert(std::abs(config.steering_forward_offset_rad - std::acos(-1.0) / 2.0) < 1e-9);
  assert(config.calibration_node_id == 1);
  assert(std::abs(config.wheel_base_m - 0.42) < 1e-9);
  assert(std::abs(config.track_width_m - 0.36) < 1e-9);
  assert(std::abs(chassis::SteeringConfigFile{}.wheel_base_m - 0.42) < 1e-9);
  assert(std::abs(chassis::SteeringConfigFile{}.track_width_m - 0.36) < 1e-9);
  assert(std::abs(config.calibration_search_speed_rpm - 2.0) < 1e-9);
  assert(std::abs(chassis::calibration_motor_search_rpm(config) - 18.0505415162) < 1e-9);
  assert(std::abs(config.calibration_torque_percent - 10.0) < 1e-9);
  assert(std::abs(config.calibration_torque_limit_nm - 10.0) < 1e-9);
  assert(std::abs(config.calibration_contact_torque_nm - 1.0) < 1e-9);
  assert(std::abs(config.calibration_min_limit_separation_rad - std::acos(-1.0)) < 1e-9);
  assert(config.calibration_position_epsilon_inc == 64);
  assert(config.calibration_stall_time_s == 0.0);
  assert(std::abs(config.calibration_current_threshold_a - 0.05) < 1e-9);
  assert(std::abs(config.calibration_current_rise_a - 1.5) < 1e-9);
  assert(std::abs(config.calibration_timeout_s - 230.0) < 1e-9);
  assert(!config.calibration_ignore_drive_fault);
  assert(!config.steering_calibrated);
  assert(std::abs(config.motor_peak_current_a - 14.3) < 1e-9);
  assert(std::abs(config.motor_torque_constant_nm_per_a - 0.1153846154) < 1e-9);
  assert(config.motor.node_ids[0] == 1);
  assert(config.motor.node_ids[1] == 2);
  assert(config.motor.node_ids[2] == 3);
  assert(config.motor.node_ids[3] == 4);
  assert((config.motor.inverted == std::array<bool, 4>{{true, true, true, true}}));
  assert((config.drive_can_buses == std::array<unsigned int, 4>{{1, 1, 2, 2}}));
  assert((config.drive_node_ids == std::array<std::uint8_t, 4>{{1, 2, 3, 4}}));
  assert((config.drive_inverted == std::array<bool, 4>{{false, false, true, true}}));
  assert(std::abs(config.drive_max_output_rpm - 105.0) < 1e-9);
  assert(std::abs(config.drive_acceleration_rpm_s - 60.0) < 1e-9);
  assert(std::abs(config.drive_deceleration_rpm_s - 120.0) < 1e-9);
  assert(std::abs(config.drive_steering_tolerance_rad - 0.1) < 1e-9);
  assert(std::abs(config.steering_limit_margin_rad - 0.0872664626) < 1e-12);
  assert(std::abs(config.steering_rate_limit_rad_s - 0.5) < 1e-9);
  assert(std::abs(config.drive_alignment_stop_rad - 0.5) < 1e-9);
  assert(std::abs(config.command_yaw_acceleration_rad_s2 - 0.8) < 1e-9);
  config.zero_offset_inc[0] = 42;
  config.steering_calibrated = true;
  config.calibration_search_speed_rpm = 0.2;
  config.calibration_contact_torque_nm = 0.15;
  config.calibration_min_limit_separation_rad = 3.2;
  config.calibration_position_epsilon_inc = 0;
  config.calibration_stall_time_s = 0.0;
  config.steering_limit_margin_rad = 0.12;
  config.steering_rate_limit_rad_s = 0.7;
  config.drive_alignment_stop_rad = 0.6;
  config.command_yaw_acceleration_rad_s2 = 1.1;
  const std::string copy = "/tmp/steering-config-test.yaml";
  chassis::save_steering_config(copy, config);
  auto reloaded = chassis::load_steering_config(copy);
  assert(reloaded.zero_offset_inc[0] == 42);
  assert(std::abs(reloaded.steering_forward_offset_rad - std::acos(-1.0) / 2.0) < 1e-9);
  assert(reloaded.steering_calibrated);
  assert(reloaded.calibration_node_id == 1);
  assert(std::abs(reloaded.calibration_search_speed_rpm - 0.2) < 1e-9);
  assert(std::abs(reloaded.calibration_torque_percent - 10.0) < 1e-9);
  assert(std::abs(reloaded.calibration_torque_limit_nm - 10.0) < 1e-9);
  assert(std::abs(reloaded.calibration_contact_torque_nm - 0.15) < 1e-9);
  assert(std::abs(reloaded.calibration_min_limit_separation_rad - 3.2) < 1e-9);
  assert(reloaded.calibration_position_epsilon_inc == 0);
  assert(reloaded.calibration_stall_time_s == 0.0);
  assert(!reloaded.calibration_ignore_drive_fault);
  assert(reloaded.drive_can_buses == config.drive_can_buses);
  assert(reloaded.drive_node_ids == config.drive_node_ids);
  assert(reloaded.drive_inverted == config.drive_inverted);
  assert(reloaded.motor.inverted == config.motor.inverted);
  assert(std::abs(reloaded.drive_max_output_rpm - config.drive_max_output_rpm) < 1e-9);
  assert(std::abs(reloaded.steering_limit_margin_rad - 0.12) < 1e-9);
  assert(std::abs(reloaded.steering_rate_limit_rad_s - 0.7) < 1e-9);
  assert(std::abs(reloaded.drive_alignment_stop_rad - 0.6) < 1e-9);
  assert(std::abs(reloaded.command_yaw_acceleration_rad_s2 - 1.1) < 1e-9);
  assert(std::abs(reloaded.motor_peak_current_a - 14.3) < 1e-9);
  assert(std::abs(reloaded.motor_torque_constant_nm_per_a - 0.1153846154) < 1e-9);
  for (double offset : {std::acos(-1.0) + 0.1, -std::acos(-1.0) - 0.1,
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
  {
    auto invalid_offset = config;
    invalid_offset.steering_forward_offset_rad = offset;
    bool rejected = false;
    try {
      chassis::validate_chassis_config(invalid_offset);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    assert(rejected);
  }
  auto no_offset = YAML::LoadFile(source);
  no_offset["chassis"]["ros__parameters"].remove("steering_forward_offset_rad");
  {
    std::ofstream output(copy);
    output << no_offset;
  }
  assert(std::abs(chassis::load_steering_config(copy).steering_forward_offset_rad) < 1e-9);
  auto old_smooth_yaml = YAML::LoadFile(source);
  old_smooth_yaml["chassis"]["ros__parameters"].remove("steering_limit_margin_rad");
  old_smooth_yaml["chassis"]["ros__parameters"].remove("steering_rate_limit_rad_s");
  old_smooth_yaml["chassis"]["ros__parameters"].remove("drive_alignment_stop_rad");
  old_smooth_yaml["chassis"]["ros__parameters"].remove("command_yaw_acceleration_rad_s2");
  {
    std::ofstream output(copy);
    output << old_smooth_yaml;
  }
  const auto old_smooth = chassis::load_steering_config(copy);
  assert(std::abs(old_smooth.steering_limit_margin_rad - 0.0872664626) < 1e-12);
  assert(std::abs(old_smooth.steering_rate_limit_rad_s - 0.5) < 1e-9);
  assert(std::abs(old_smooth.drive_alignment_stop_rad - 0.5) < 1e-9);
  assert(std::abs(old_smooth.command_yaw_acceleration_rad_s2 - 0.8) < 1e-9);
  // An old overspeed value must never become the new velocity target.
  auto legacy = YAML::LoadFile(source);
  legacy["chassis"]["ros__parameters"].remove("calibration_search_speed_rpm");
  legacy["chassis"]["ros__parameters"].remove("calibration_contact_torque_nm");
  legacy["chassis"]["ros__parameters"].remove("calibration_min_limit_separation_rad");
  legacy["chassis"]["ros__parameters"]["calibration_speed_rpm"] = 100.0;
  {
    std::ofstream output(copy);
    output << legacy;
  }
  const auto migrated = chassis::load_steering_config(copy);
  assert(std::abs(migrated.calibration_search_speed_rpm - 0.1) < 1e-9);
  assert(std::abs(chassis::calibration_motor_search_rpm(migrated) - 0.90252707581) < 1e-9);
  assert(std::abs(migrated.calibration_min_limit_separation_rad - std::acos(-1.0)) < 1e-9);
  assert(
    std::abs(
      migrated.calibration_contact_torque_nm -
      migrated.calibration_current_threshold_a * migrated.motor_torque_constant_nm_per_a *
      migrated.motor.gear_ratio) < 1e-9);

  for (double threshold : {0.0, -0.1, 10.0, 10.1,
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
  {
    auto invalid_contact = config;
    invalid_contact.calibration_contact_torque_nm = threshold;
    chassis::save_steering_config(copy, invalid_contact);
    bool rejected = false;
    try {
      (void)chassis::load_steering_config(copy);
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    assert(rejected);
  }

  for (double separation : {0.0, -0.1, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    auto invalid_separation = config;
    invalid_separation.calibration_min_limit_separation_rad = separation;
    chassis::save_steering_config(copy, invalid_separation);
    bool rejected = false;
    try {
      (void)chassis::load_steering_config(copy);
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    assert(rejected);
  }

  const auto rejects_config = [](const chassis::SteeringConfigFile & candidate) {
      bool rejected = false;
      try {
        chassis::validate_chassis_config(candidate);
      } catch (const std::invalid_argument &) {
        rejected = true;
      }
      assert(rejected);
    };
  auto invalid = config;
  for (double value : {0.0, -0.1, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    invalid = config;
    invalid.steering_limit_margin_rad = value;
    rejects_config(invalid);
    invalid = config;
    invalid.steering_rate_limit_rad_s = value;
    rejects_config(invalid);
    invalid = config;
    invalid.drive_alignment_stop_rad = value;
    rejects_config(invalid);
    invalid = config;
    invalid.command_yaw_acceleration_rad_s2 = value;
    rejects_config(invalid);
  }
  invalid = config;
  invalid.steering_limit_margin_rad = invalid.max_steering_angle_rad;
  rejects_config(invalid);
  invalid = config;
  invalid.drive_alignment_stop_rad = invalid.drive_steering_tolerance_rad;
  rejects_config(invalid);
  invalid = config;
  invalid.drive_alignment_stop_rad = std::acos(-1.0) / 2.0 + 0.01;
  rejects_config(invalid);

  const auto rejects = [](const chassis::SteeringConfigFile & candidate) {
      bool rejected = false;
      try {
        (void)chassis::calibration_motor_search_rpm(candidate);
      } catch (const std::invalid_argument &) {
        rejected = true;
      }
      assert(rejected);
    };
  for (double speed : {0.0, -0.1, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), 100.0})
  {
    auto invalid = migrated;
    invalid.calibration_search_speed_rpm = speed;
    rejects(invalid);
  }
  for (double ratio : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    auto invalid = migrated;
    invalid.motor.gear_ratio = ratio;
    rejects(invalid);
  }
  invalid = migrated;
  invalid.motor.encoder_resolution = 0;
  rejects(invalid);
  invalid = migrated;
  invalid.calibration_speed_rpm = chassis::calibration_motor_search_rpm(migrated);
  rejects(invalid);
  std::remove(copy.c_str());
  return 0;
}
