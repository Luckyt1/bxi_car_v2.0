#pragma once

#include "iswv/iswv.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace chassis
{

struct SteeringConfigFile
{
  std::int64_t can_bus{0};
  double wheel_base_m{0.42};
  double track_width_m{0.36};
  double max_steering_angle_rad{3.14159265358979323846};
  double steering_forward_offset_rad{0.0};
  double command_timeout_s{0.5};
  bool steering_calibrated{false};
  double calibration_speed_rpm{20.0};
  double calibration_search_speed_rpm{0.1};
  double calibration_torque_percent{5.0};
  double calibration_torque_limit_nm{0.5};
  double calibration_contact_torque_nm{0.052};
  double calibration_min_limit_separation_rad{3.14159265358979323846};
  std::int32_t calibration_position_epsilon_inc{64};
  double calibration_stall_time_s{0.0};
  double calibration_current_threshold_a{0.05};
  double calibration_current_rise_a{1.5};
  double calibration_timeout_s{30.0};
  std::uint8_t calibration_node_id{1};
  bool calibration_ignore_drive_fault{false};
  double motor_peak_current_a{14.3};
  double motor_torque_constant_nm_per_a{0.75 / 6.5};
  std::array<unsigned int, 4> drive_can_buses{{1, 1, 2, 2}};
  std::array<std::uint8_t, 4> drive_node_ids{{1, 2, 3, 4}};
  std::array<bool, 4> drive_inverted{};
  std::uint32_t drive_encoder_resolution{65536};
  double drive_gear_ratio{10.0};
  double drive_max_output_rpm{105.0};
  double drive_acceleration_rpm_s{60.0};
  double drive_deceleration_rpm_s{120.0};
  double drive_steering_tolerance_rad{0.1};
  double steering_limit_margin_rad{0.0872664626};
  double steering_rate_limit_rad_s{0.5};
  double drive_alignment_stop_rad{0.5};
  double command_yaw_acceleration_rad_s2{0.8};
  iswv::SteeringLayoutConfig motor{};
  std::array<std::int32_t, 4> zero_offset_inc{};
};

inline void validate_chassis_config(const SteeringConfigFile & config)
{
  auto positive = [](double value) {return std::isfinite(value) && value > 0.0;};
  if (config.can_bus != 0 || !positive(config.wheel_base_m) ||
    !positive(config.track_width_m) || !positive(config.command_timeout_s) ||
    !positive(config.max_steering_angle_rad) || config.max_steering_angle_rad > std::acos(-1.0) ||
    !std::isfinite(config.steering_forward_offset_rad) ||
    std::abs(config.steering_forward_offset_rad) > std::acos(-1.0) ||
    !positive(config.motor.gear_ratio) || config.motor.encoder_resolution == 0 ||
    !positive(config.motor.wheel_diameter_m) || !positive(config.motor.profile_velocity_rpm) ||
    !positive(config.motor.acceleration_rps2) || !positive(config.motor.deceleration_rps2) ||
    !positive(config.drive_gear_ratio) || config.drive_encoder_resolution == 0 ||
    !positive(config.drive_max_output_rpm) || !positive(config.drive_acceleration_rpm_s) ||
    !positive(config.drive_deceleration_rpm_s) || !positive(config.drive_steering_tolerance_rad) ||
    !positive(config.steering_limit_margin_rad) ||
    config.steering_limit_margin_rad >= config.max_steering_angle_rad ||
    !positive(config.steering_rate_limit_rad_s) ||
    !positive(config.drive_alignment_stop_rad) ||
    config.drive_alignment_stop_rad <= config.drive_steering_tolerance_rad ||
    config.drive_alignment_stop_rad > std::acos(-1.0) / 2.0 ||
    !positive(config.command_yaw_acceleration_rad_s2) ||
    !positive(config.calibration_speed_rpm) || !positive(config.calibration_search_speed_rpm) ||
    !positive(config.calibration_timeout_s) || !positive(
      config.calibration_min_limit_separation_rad) ||
    !positive(config.calibration_contact_torque_nm) ||
    !positive(config.calibration_torque_limit_nm) ||
    config.calibration_contact_torque_nm >= config.calibration_torque_limit_nm ||
    !positive(config.motor_peak_current_a) || !positive(config.motor_torque_constant_nm_per_a) ||
    config.calibration_ignore_drive_fault || config.calibration_position_epsilon_inc < 0 ||
    !std::isfinite(config.calibration_stall_time_s) || config.calibration_stall_time_s < 0.0)
  {
    throw std::invalid_argument("invalid chassis geometry, ISWV units or calibration limits");
  }
  const double search_rpm = config.calibration_search_speed_rpm * config.motor.gear_ratio;
  if (!std::isfinite(search_rpm) || search_rpm >= config.calibration_speed_rpm) {
    throw std::invalid_argument("calibration search exceeds motor RPM limit");
  }
  std::set<std::uint8_t> steering_nodes;
  std::set<std::pair<unsigned int, std::uint8_t>> travel_nodes;
  for (std::size_t i = 0; i < 4; ++i) {
    const auto steering_id = config.motor.node_ids[i];
    const auto travel_id = config.drive_node_ids[i];
    if (steering_id < 1 || steering_id > 127 || !steering_nodes.insert(steering_id).second ||
      travel_id < 1 || travel_id > 127 ||
      (config.drive_can_buses[i] != 1 && config.drive_can_buses[i] != 2) ||
      !travel_nodes.emplace(config.drive_can_buses[i], travel_id).second)
    {
      throw std::invalid_argument(
              "CAN0 requires unique steering IDs; CAN1/2 require unique travel IDs per bus");
    }
  }
}

inline YAML::Node steering_parameters(const YAML::Node & document)
{
  const auto node = document["chassis"];
  if (!node || !node["ros__parameters"]) {
    throw std::runtime_error("YAML must contain chassis.ros__parameters");
  }
  return node["ros__parameters"];
}

template<typename T>
T steering_value(const YAML::Node & parameters, const char * name)
{
  const auto value = parameters[name];
  if (!value) {
    throw std::runtime_error(std::string("missing steering parameter: ") + name);
  }
  try {
    return value.as<T>();
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            std::string("invalid steering parameter ") + name + ": " +
            error.what());
  }
}

template<typename T>
T steering_value_or(const YAML::Node & parameters, const char * name, const T & fallback)
{
  const auto value = parameters[name];
  if (!value) {
    return fallback;
  }
  try {
    return value.as<T>();
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            std::string("invalid steering parameter ") + name + ": " +
            error.what());
  }
}

inline SteeringConfigFile load_steering_config(const std::string & path)
{
  SteeringConfigFile config;
  try {
    const auto parameters = steering_parameters(YAML::LoadFile(path));
    config.can_bus = steering_value<std::int64_t>(parameters, "can_bus");
    config.wheel_base_m = steering_value<double>(parameters, "wheel_base_m");
    config.track_width_m = steering_value<double>(parameters, "track_width_m");
    config.max_steering_angle_rad = steering_value<double>(parameters, "max_steering_angle_rad");
    config.steering_forward_offset_rad = steering_value_or<double>(
      parameters, "steering_forward_offset_rad", config.steering_forward_offset_rad);
    config.command_timeout_s = steering_value<double>(parameters, "command_timeout_s");
    config.steering_calibrated = steering_value_or<bool>(
      parameters, "steering_calibrated", false);
    config.calibration_speed_rpm = steering_value<double>(parameters, "calibration_speed_rpm");
    config.calibration_search_speed_rpm = steering_value_or<double>(
      parameters, "calibration_search_speed_rpm", config.calibration_search_speed_rpm);
    config.calibration_torque_percent = steering_value_or<double>(
      parameters, "calibration_torque_percent", config.calibration_torque_percent);
    config.calibration_torque_limit_nm = steering_value_or<double>(
      parameters, "calibration_torque_limit_nm", config.calibration_torque_limit_nm);
    config.calibration_min_limit_separation_rad = steering_value_or<double>(
      parameters, "calibration_min_limit_separation_rad",
      config.calibration_min_limit_separation_rad);
    config.calibration_position_epsilon_inc = steering_value_or<std::int32_t>(
      parameters, "calibration_position_epsilon_inc", config.calibration_position_epsilon_inc);
    config.calibration_stall_time_s = steering_value_or<double>(
      parameters, "calibration_stall_time_s", config.calibration_stall_time_s);
    config.calibration_current_threshold_a = steering_value_or<double>(
      parameters, "calibration_current_threshold_a", config.calibration_current_threshold_a);
    config.calibration_current_rise_a = steering_value_or<double>(
      parameters, "calibration_current_rise_a", config.calibration_current_rise_a);
    config.calibration_timeout_s = steering_value_or<double>(
      parameters, "calibration_timeout_s", config.calibration_timeout_s);
    const auto calibration_node_id =
      steering_value<std::uint32_t>(parameters, "calibration_node_id");
    config.calibration_ignore_drive_fault =
      steering_value<bool>(parameters, "calibration_ignore_drive_fault");
    config.motor_peak_current_a = steering_value<double>(parameters, "motor_peak_current_a");
    config.motor_torque_constant_nm_per_a =
      steering_value<double>(parameters, "motor_torque_constant_nm_per_a");
    config.drive_max_output_rpm = steering_value_or<double>(
      parameters, "drive_max_output_rpm", config.drive_max_output_rpm);
    config.drive_acceleration_rpm_s = steering_value_or<double>(
      parameters, "drive_acceleration_rpm_s", config.drive_acceleration_rpm_s);
    config.drive_deceleration_rpm_s = steering_value_or<double>(
      parameters, "drive_deceleration_rpm_s", config.drive_deceleration_rpm_s);
    config.drive_steering_tolerance_rad = steering_value_or<double>(
      parameters, "drive_steering_tolerance_rad", config.drive_steering_tolerance_rad);
    config.steering_limit_margin_rad = steering_value_or<double>(
      parameters, "steering_limit_margin_rad", config.steering_limit_margin_rad);
    config.steering_rate_limit_rad_s = steering_value_or<double>(
      parameters, "steering_rate_limit_rad_s", config.steering_rate_limit_rad_s);
    config.drive_alignment_stop_rad = steering_value_or<double>(
      parameters, "drive_alignment_stop_rad", config.drive_alignment_stop_rad);
    config.command_yaw_acceleration_rad_s2 = steering_value_or<double>(
      parameters, "command_yaw_acceleration_rad_s2", config.command_yaw_acceleration_rad_s2);
    config.drive_encoder_resolution = steering_value_or<std::uint32_t>(
      parameters, "drive_encoder_resolution", config.drive_encoder_resolution);
    config.drive_gear_ratio = steering_value_or<double>(
      parameters, "drive_gear_ratio", config.drive_gear_ratio);
    if (calibration_node_id < 1 || calibration_node_id > 127) {
      throw std::runtime_error("calibration_node_id must be in range 1..127");
    }
    if (!std::isfinite(config.calibration_torque_percent) ||
      config.calibration_torque_percent <= 0.0 || config.calibration_torque_percent > 100.0 ||
      !std::isfinite(config.calibration_torque_limit_nm) ||
      config.calibration_torque_limit_nm <= 0.0 ||
      !std::isfinite(config.calibration_current_threshold_a) ||
      config.calibration_current_threshold_a <= 0.0 ||
      !std::isfinite(config.calibration_current_rise_a) ||
      config.calibration_current_rise_a <= 0.0 ||
      !std::isfinite(config.calibration_timeout_s) || config.calibration_timeout_s <= 0.0 ||
      !std::isfinite(config.motor_peak_current_a) || config.motor_peak_current_a <= 0.0 ||
      !std::isfinite(config.motor_torque_constant_nm_per_a) ||
      config.motor_torque_constant_nm_per_a <= 0.0 ||
      !std::isfinite(config.drive_max_output_rpm) || config.drive_max_output_rpm <= 0.0 ||
      !std::isfinite(config.drive_acceleration_rpm_s) ||
      config.drive_acceleration_rpm_s <= 0.0 ||
      !std::isfinite(config.drive_deceleration_rpm_s) ||
      config.drive_deceleration_rpm_s <= 0.0 ||
      !std::isfinite(config.drive_steering_tolerance_rad) ||
      config.drive_steering_tolerance_rad <= 0.0 ||
      !std::isfinite(config.steering_limit_margin_rad) ||
      config.steering_limit_margin_rad <= 0.0 ||
      !std::isfinite(config.steering_rate_limit_rad_s) ||
      config.steering_rate_limit_rad_s <= 0.0 ||
      !std::isfinite(config.drive_alignment_stop_rad) ||
      config.drive_alignment_stop_rad <= 0.0 ||
      !std::isfinite(config.command_yaw_acceleration_rad_s2) ||
      config.command_yaw_acceleration_rad_s2 <= 0.0)
    {
      throw std::runtime_error(
              "motor current, torque conversion and smooth control parameters must be positive");
    }
    if (!std::isfinite(config.calibration_min_limit_separation_rad) ||
      config.calibration_min_limit_separation_rad <= 0.0)
    {
      throw std::runtime_error("calibration_min_limit_separation_rad must be positive");
    }
    config.calibration_node_id = static_cast<std::uint8_t>(calibration_node_id);
    config.motor.encoder_resolution =
      steering_value<std::uint32_t>(parameters, "encoder_resolution");
    config.motor.gear_ratio = steering_value<double>(parameters, "gear_ratio");
    config.calibration_contact_torque_nm = steering_value_or<double>(
      parameters, "calibration_contact_torque_nm",
      config.calibration_current_threshold_a *
      config.motor_torque_constant_nm_per_a * config.motor.gear_ratio);
    if (!std::isfinite(config.calibration_contact_torque_nm) ||
      config.calibration_contact_torque_nm <= 0.0 ||
      config.calibration_contact_torque_nm >= config.calibration_torque_limit_nm)
    {
      throw std::runtime_error(
              "calibration_contact_torque_nm must be positive and below calibration_torque_limit_nm");
    }
    config.motor.wheel_diameter_m = steering_value<double>(parameters, "wheel_diameter_m");
    config.motor.profile_velocity_rpm = steering_value<double>(parameters, "profile_velocity_rpm");
    config.motor.acceleration_rps2 = steering_value<double>(parameters, "acceleration_rps2");
    config.motor.deceleration_rps2 = steering_value<double>(parameters, "deceleration_rps2");
    const auto rpdo_type = steering_value<std::uint32_t>(parameters, "rpdo_transmission_type");
    const auto tpdo_type = steering_value<std::uint32_t>(parameters, "tpdo_transmission_type");
    const auto inhibit_time = steering_value<std::uint32_t>(parameters, "tpdo_inhibit_time");
    const auto event_timer = steering_value<std::uint32_t>(parameters, "tpdo_event_timer_ms");
    if (rpdo_type > 255 || tpdo_type > 255 || inhibit_time > 65535 || event_timer > 65535) {
      throw std::runtime_error("invalid PDO parameter range");
    }
    config.motor.pdo.rpdo_transmission_type = static_cast<std::uint8_t>(rpdo_type);
    config.motor.pdo.tpdo_transmission_type = static_cast<std::uint8_t>(tpdo_type);
    config.motor.pdo.tpdo_inhibit_time = static_cast<std::uint16_t>(inhibit_time);
    config.motor.pdo.tpdo_event_timer_ms = static_cast<std::uint16_t>(event_timer);

    const auto node_ids = steering_value<std::vector<int>>(parameters, "node_ids");
    const auto inverted = steering_value<std::vector<bool>>(parameters, "inverted");
    const auto offsets = steering_value<std::vector<std::int64_t>>(parameters, "zero_offset_inc");
    const auto drive_can_buses = steering_value_or<std::vector<int>>(
      parameters, "drive_can_buses", std::vector<int>{1, 1, 2, 2});
    const auto drive_node_ids = steering_value_or<std::vector<int>>(
      parameters, "drive_node_ids", std::vector<int>{1, 2, 3, 4});
    const auto drive_inverted = steering_value_or<std::vector<bool>>(
      parameters, "drive_inverted", std::vector<bool>{false, false, false, false});
    if (node_ids.size() != 4 || inverted.size() != 4 || offsets.size() != 4) {
      throw std::runtime_error("node_ids, inverted and zero_offset_inc must contain four values");
    }
    if (drive_can_buses.size() != 4 || drive_node_ids.size() != 4 ||
      drive_inverted.size() != 4)
    {
      throw std::runtime_error(
              "drive_can_buses, drive_node_ids and drive_inverted must contain four values");
    }
    for (std::size_t i = 0; i < 4; ++i) {
      if (node_ids[i] < 1 || node_ids[i] > 127 ||
        offsets[i] < std::numeric_limits<std::int32_t>::min() ||
        offsets[i] > std::numeric_limits<std::int32_t>::max())
      {
        throw std::runtime_error("invalid steering node id or zero offset");
      }
      config.motor.node_ids[i] = static_cast<std::uint8_t>(node_ids[i]);
      config.motor.inverted[i] = inverted[i];
      config.zero_offset_inc[i] = static_cast<std::int32_t>(offsets[i]);
      if (drive_can_buses[i] < 0 || drive_can_buses[i] >= 7 ||
        drive_node_ids[i] < 1 || drive_node_ids[i] > 127)
      {
        throw std::runtime_error("invalid drive CAN bus or node id");
      }
      config.drive_can_buses[i] = static_cast<unsigned int>(drive_can_buses[i]);
      config.drive_node_ids[i] = static_cast<std::uint8_t>(drive_node_ids[i]);
      config.drive_inverted[i] = drive_inverted[i];
    }
    if (std::find(
        config.motor.node_ids.begin(), config.motor.node_ids.end(),
        config.calibration_node_id) == config.motor.node_ids.end())
    {
      throw std::runtime_error("calibration_node_id is not present in node_ids");
    }
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("cannot load steering YAML " + path + ": " + error.what());
  }
  validate_chassis_config(config);
  return config;
}

// The search setting is output-shaft RPM; the driver and overspeed guard use motor RPM.
// Validate before powering the motors, including legacy YAML with no search setting.
inline double calibration_motor_search_rpm(const SteeringConfigFile & config)
{
  const double rpm = config.calibration_search_speed_rpm * config.motor.gear_ratio;
  if (!std::isfinite(config.calibration_search_speed_rpm) ||
    config.calibration_search_speed_rpm <= 0.0 ||
    !std::isfinite(config.motor.gear_ratio) || config.motor.gear_ratio <= 0.0 ||
    config.motor.encoder_resolution == 0 ||
    !std::isfinite(config.calibration_speed_rpm) || config.calibration_speed_rpm <= 0.0 ||
    !std::isfinite(rpm) || rpm >= config.calibration_speed_rpm)
  {
    throw std::invalid_argument(
            "calibration_search_speed_rpm must be positive output RPM; "
            "search RPM * gear_ratio must be below calibration_speed_rpm (motor RPM)");
  }
  return rpm;
}

inline void save_steering_config(const std::string & path, const SteeringConfigFile & config)
{
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("cannot write steering YAML: " + path);
  }
  output << std::setprecision(17);
  output << "chassis:\n  ros__parameters:\n"
         << "    can_bus: " << config.can_bus << "\n"
         << "    wheel_base_m: " << config.wheel_base_m << "\n"
         << "    track_width_m: " << config.track_width_m << "\n"
         << "    max_steering_angle_rad: " << config.max_steering_angle_rad << "\n"
         << "    steering_forward_offset_rad: " << config.steering_forward_offset_rad << "\n"
         << "    command_timeout_s: " << config.command_timeout_s << "\n"
         << "    steering_calibrated: " << (config.steering_calibrated ? "true" : "false") << "\n"
         << "    calibration_speed_rpm: " << config.calibration_speed_rpm << "\n"
         << "    calibration_search_speed_rpm: " << config.calibration_search_speed_rpm << "\n"
         << "    calibration_torque_percent: " << config.calibration_torque_percent << "\n"
         << "    calibration_torque_limit_nm: " << config.calibration_torque_limit_nm << "\n"
         << "    calibration_contact_torque_nm: " << config.calibration_contact_torque_nm << "\n"
         << "    calibration_min_limit_separation_rad: "
         << config.calibration_min_limit_separation_rad << "\n"
         << "    calibration_position_epsilon_inc: " << config.calibration_position_epsilon_inc <<
    "\n"
         << "    calibration_stall_time_s: " << config.calibration_stall_time_s << "\n"
         << "    calibration_current_threshold_a: " << config.calibration_current_threshold_a <<
    "\n"
         << "    calibration_current_rise_a: " << config.calibration_current_rise_a << "\n"
         << "    calibration_timeout_s: " << config.calibration_timeout_s << "\n"
         << "    calibration_node_id: " << static_cast<unsigned>(config.calibration_node_id) << "\n"
         << "    calibration_ignore_drive_fault: "
         << (config.calibration_ignore_drive_fault ? "true" : "false") << "\n"
         << "    motor_peak_current_a: " << config.motor_peak_current_a << "\n"
         << "    motor_torque_constant_nm_per_a: "
         << config.motor_torque_constant_nm_per_a << "\n"
         << "    drive_max_output_rpm: " << config.drive_max_output_rpm << "\n"
         << "    drive_encoder_resolution: " << config.drive_encoder_resolution << "\n"
         << "    drive_gear_ratio: " << config.drive_gear_ratio << "\n"
         << "    drive_acceleration_rpm_s: " << config.drive_acceleration_rpm_s << "\n"
         << "    drive_deceleration_rpm_s: " << config.drive_deceleration_rpm_s << "\n"
         << "    drive_steering_tolerance_rad: " << config.drive_steering_tolerance_rad << "\n"
         << "    steering_limit_margin_rad: " << config.steering_limit_margin_rad << "\n"
         << "    steering_rate_limit_rad_s: " << config.steering_rate_limit_rad_s << "\n"
         << "    drive_alignment_stop_rad: " << config.drive_alignment_stop_rad << "\n"
         << "    command_yaw_acceleration_rad_s2: "
         << config.command_yaw_acceleration_rad_s2 << "\n"
         << "    encoder_resolution: " << config.motor.encoder_resolution << "\n"
         << "    gear_ratio: " << config.motor.gear_ratio << "\n"
         << "    wheel_diameter_m: " << config.motor.wheel_diameter_m << "\n"
         << "    profile_velocity_rpm: " << config.motor.profile_velocity_rpm << "\n"
         << "    acceleration_rps2: " << config.motor.acceleration_rps2 << "\n"
         << "    deceleration_rps2: " << config.motor.deceleration_rps2 << "\n"
         << "    rpdo_transmission_type: " <<
    static_cast<unsigned>(config.motor.pdo.rpdo_transmission_type) << "\n"
         << "    tpdo_transmission_type: " <<
    static_cast<unsigned>(config.motor.pdo.tpdo_transmission_type) << "\n"
         << "    tpdo_inhibit_time: " << config.motor.pdo.tpdo_inhibit_time << "\n"
         << "    tpdo_event_timer_ms: " << config.motor.pdo.tpdo_event_timer_ms << "\n"
         << "    node_ids: [";
  for (std::size_t i = 0; i < 4; ++i) {
    output << (i ? ", " : "") << static_cast<unsigned>(config.motor.node_ids[i]);
  }
  output << "]\n    inverted: [";
  for (std::size_t i = 0; i < 4; ++i) {
    output << (i ? ", " : "") << (config.motor.inverted[i] ? "true" : "false");
  }
  output << "]\n    zero_offset_inc: [";
  for (std::size_t i = 0; i < 4; ++i) {
    output << (i ? ", " : "") << config.zero_offset_inc[i];
  }
  output << "]\n    drive_can_buses: [";
  for (std::size_t i = 0; i < 4; ++i) {
    output << (i ? ", " : "") << config.drive_can_buses[i];
  }
  output << "]\n    drive_node_ids: [";
  for (std::size_t i = 0; i < 4; ++i) {
    output << (i ? ", " : "") << static_cast<unsigned>(config.drive_node_ids[i]);
  }
  output << "]\n    drive_inverted: [";
  for (std::size_t i = 0; i < 4; ++i) {
    output << (i ? ", " : "") << (config.drive_inverted[i] ? "true" : "false");
  }
  output << "]\n";
  if (!output) {throw std::runtime_error("failed while writing steering YAML: " + path);}
}

}  // namespace chassis
