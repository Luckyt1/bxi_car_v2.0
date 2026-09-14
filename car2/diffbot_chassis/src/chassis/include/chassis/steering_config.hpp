#pragma once

#include "iswv/iswv.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace chassis
{

struct SwerveModuleConfig
{
  double x_m{0.0};
  double y_m{0.0};
  double wheel_radius_m{0.065};
  double steering_gear_ratio{2500.0 / 277.0};
  std::uint32_t steering_encoder_resolution{65536};
  int steering_sign{1};
  double heading_offset_rad{0.0};
  double steering_min_rad{-3.14159265358979323846};
  double steering_max_rad{3.14159265358979323846};
  double soft_margin_rad{0.0872664626};
  double drive_gear_ratio{10.0};
  std::uint32_t drive_encoder_resolution{65536};
  bool steering_inverted{false};
  bool drive_inverted{false};
  double drive_max_motor_rpm{1050.0};
  double steering_rate_limit_rad_s{0.5};
  double drive_acceleration_m_s2{0.4084070449666731};
  double drive_deceleration_m_s2{0.8168140899333463};
};

struct SteeringConfigFile
{
  // Runtime source path; not serialized as a motor parameter.
  std::string source_path;
  std::int64_t can_bus{0};
  double wheel_base_m{0.42};
  double track_width_m{0.36};
  double max_steering_angle_rad{3.14159265358979323846};
  double steering_forward_offset_rad{0.0};
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
  // Legacy YAML round-trip compatibility only; continuous steering does not use this threshold.
  double drive_alignment_stop_rad{0.5};
  double swerve_speed_epsilon_m_s{1e-4};
  double swerve_branch_hysteresis_rad{0.05};
  std::string drive_alignment_policy{"pause_all"};
  double drive_alignment_pause_rad{0.35};
  double command_yaw_acceleration_rad_s2{0.8};
  std::array<std::optional<int>, 4> steering_velocity_loop_kp{};
  std::array<std::optional<int>, 4> steering_velocity_loop_ki{};
  std::array<std::optional<int>, 4> steering_velocity_feedback_filter{};
  std::array<std::optional<int>, 4> steering_position_loop_kp{};
  std::array<std::optional<int>, 4> steering_position_smoothing_filter{};
  std::array<std::optional<int>, 4> drive_velocity_loop_kp{};
  std::array<std::optional<int>, 4> drive_velocity_loop_ki{};
  std::array<std::optional<int>, 4> drive_velocity_feedback_filter{};
  std::array<std::optional<int>, 4> drive_position_loop_kp{};
  std::array<std::optional<int>, 4> drive_position_smoothing_filter{};
  iswv::SteeringLayoutConfig motor{};
  std::array<std::int32_t, 4> zero_offset_inc{};
  std::optional<std::array<SwerveModuleConfig, 4>> modules{};
};

inline double drive_output_rpm_s_to_m_s2(double output_rpm_s, double wheel_radius_m)
{
  return output_rpm_s * 2.0 * std::acos(-1.0) * wheel_radius_m / 60.0;
}

inline SwerveModuleConfig legacy_module_config(
  const SteeringConfigFile & config, std::size_t index)
{
  const double half_base = 0.5 * config.wheel_base_m;
  const double half_track = 0.5 * config.track_width_m;
  const std::array<double, 4> x{{half_base, -half_base, -half_base, half_base}};
  const std::array<double, 4> y{{half_track, half_track, -half_track, -half_track}};
  const double wheel_radius = 0.5 * config.motor.wheel_diameter_m;
  SwerveModuleConfig module;
  module.x_m = x[index];
  module.y_m = y[index];
  module.wheel_radius_m = wheel_radius;
  module.steering_gear_ratio = config.motor.gear_ratio;
  module.steering_encoder_resolution = config.motor.encoder_resolution;
  module.steering_sign = 1;
  module.heading_offset_rad = -config.steering_forward_offset_rad;
  module.steering_min_rad = -config.max_steering_angle_rad;
  module.steering_max_rad = config.max_steering_angle_rad;
  module.soft_margin_rad = config.steering_limit_margin_rad;
  module.drive_gear_ratio = config.drive_gear_ratio;
  module.drive_encoder_resolution = config.drive_encoder_resolution;
  module.steering_inverted = config.motor.inverted[index];
  module.drive_inverted = config.drive_inverted[index];
  module.drive_max_motor_rpm = config.drive_max_output_rpm * config.drive_gear_ratio;
  module.steering_rate_limit_rad_s = config.steering_rate_limit_rad_s;
  module.drive_acceleration_m_s2 =
    drive_output_rpm_s_to_m_s2(config.drive_acceleration_rpm_s, wheel_radius);
  module.drive_deceleration_m_s2 =
    drive_output_rpm_s_to_m_s2(config.drive_deceleration_rpm_s, wheel_radius);
  return module;
}

inline std::array<SwerveModuleConfig, 4> resolved_modules(const SteeringConfigFile & config)
{
  if (config.modules) {
    return *config.modules;
  }
  return {
    legacy_module_config(config, 0),
    legacy_module_config(config, 1),
    legacy_module_config(config, 2),
    legacy_module_config(config, 3)};
}

inline void validate_optional_steering_raw_parameter(
  const std::array<std::optional<int>, 4> & values,
  const char * name,
  int minimum,
  int maximum)
{
  for (const auto & value : values) {
    if (value && (*value < minimum || *value > maximum)) {
      throw std::invalid_argument(
              std::string(name) + " must be null or raw integer in range " +
              std::to_string(minimum) + ".." + std::to_string(maximum));
    }
  }
}

inline void validate_chassis_config(const SteeringConfigFile & config)
{
  auto positive = [](double value) {return std::isfinite(value) && value > 0.0;};
  auto nonnegative = [](double value) {return std::isfinite(value) && value >= 0.0;};
  if (config.can_bus != 0 || !positive(config.wheel_base_m) ||
    !positive(config.track_width_m) ||
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
    !positive(config.swerve_speed_epsilon_m_s) ||
    !nonnegative(config.swerve_branch_hysteresis_rad) ||
    (config.drive_alignment_policy != "none" &&
    config.drive_alignment_policy != "pause_all" && config.drive_alignment_policy != "cosine") ||
    !positive(config.drive_alignment_pause_rad) ||
    config.drive_alignment_pause_rad <= config.drive_steering_tolerance_rad ||
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
  const auto modules = resolved_modules(config);
  for (const auto & module : modules) {
    if (!std::isfinite(module.x_m) || !std::isfinite(module.y_m) ||
      !positive(module.wheel_radius_m) || !positive(module.steering_gear_ratio) ||
      module.steering_encoder_resolution == 0 ||
      (module.steering_sign != -1 && module.steering_sign != 1) ||
      !std::isfinite(module.heading_offset_rad) ||
      !std::isfinite(module.steering_min_rad) || !std::isfinite(module.steering_max_rad) ||
      module.steering_min_rad >= module.steering_max_rad ||
      !nonnegative(module.soft_margin_rad) ||
      module.steering_min_rad + module.soft_margin_rad >=
      module.steering_max_rad - module.soft_margin_rad ||
      !positive(module.drive_gear_ratio) || module.drive_encoder_resolution == 0 ||
      !positive(module.drive_max_motor_rpm) ||
      !positive(module.steering_rate_limit_rad_s) ||
      !positive(module.drive_acceleration_m_s2) ||
      !positive(module.drive_deceleration_m_s2))
    {
      throw std::invalid_argument("invalid swerve module geometry, limits or units");
    }
    const double module_search_rpm =
      config.calibration_search_speed_rpm * module.steering_gear_ratio;
    if (!std::isfinite(module_search_rpm) || module_search_rpm >= config.calibration_speed_rpm) {
      throw std::invalid_argument("calibration search exceeds a module motor RPM limit");
    }
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
  validate_optional_steering_raw_parameter(
    config.steering_velocity_loop_kp, "steering_velocity_loop_kp", 1, 32767);
  validate_optional_steering_raw_parameter(
    config.steering_velocity_loop_ki, "steering_velocity_loop_ki", 0, 1023);
  validate_optional_steering_raw_parameter(
    config.steering_velocity_feedback_filter, "steering_velocity_feedback_filter", 0, 45);
  validate_optional_steering_raw_parameter(
    config.steering_position_loop_kp, "steering_position_loop_kp", 0, 32767);
  validate_optional_steering_raw_parameter(
    config.steering_position_smoothing_filter, "steering_position_smoothing_filter", 1, 255);
  validate_optional_steering_raw_parameter(
    config.drive_velocity_loop_kp, "drive_velocity_loop_kp", 1, 32767);
  validate_optional_steering_raw_parameter(
    config.drive_velocity_loop_ki, "drive_velocity_loop_ki", 0, 1023);
  validate_optional_steering_raw_parameter(
    config.drive_velocity_feedback_filter, "drive_velocity_feedback_filter", 0, 45);
  validate_optional_steering_raw_parameter(
    config.drive_position_loop_kp, "drive_position_loop_kp", 0, 32767);
  validate_optional_steering_raw_parameter(
    config.drive_position_smoothing_filter, "drive_position_smoothing_filter", 1, 255);
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

inline std::int64_t steering_raw_integer(const YAML::Node & value, const char * name)
{
  if (!value.IsScalar()) {
    throw std::runtime_error(
            std::string("invalid steering parameter ") + name + ": expected integer");
  }
  const std::string text = value.Scalar();
  if (text.empty()) {
    throw std::runtime_error(
            std::string("invalid steering parameter ") + name + ": expected integer");
  }
  std::int64_t parsed = 0;
  const auto * begin = text.data();
  const auto * end = begin + text.size();
  const auto result = std::from_chars(begin, end, parsed);
  if (result.ec == std::errc::result_out_of_range) {
    throw std::runtime_error(
            std::string("invalid steering parameter ") + name + ": integer out of range");
  }
  if (result.ec != std::errc{} || result.ptr != end) {
    throw std::runtime_error(
            std::string("invalid steering parameter ") + name + ": expected integer");
  }
  return parsed;
}

inline std::optional<int> steering_optional_raw_integer(
  const YAML::Node & value,
  const char * name,
  int minimum,
  int maximum)
{
  if (value.IsNull()) {
    return std::nullopt;
  }
  const auto parsed = steering_raw_integer(value, name);
  if (parsed < minimum || parsed > maximum) {
    throw std::runtime_error(
            std::string("invalid steering parameter ") + name + ": value must be in range " +
            std::to_string(minimum) + ".." + std::to_string(maximum));
  }
  return static_cast<int>(parsed);
}

inline std::array<std::optional<int>, 4> steering_optional_raw_parameter(
  const YAML::Node & parameters,
  const char * name,
  int minimum,
  int maximum)
{
  std::array<std::optional<int>, 4> values{};
  const auto node = parameters[name];
  if (!node || node.IsNull()) {
    return values;
  }
  if (node.IsScalar()) {
    values.fill(steering_optional_raw_integer(node, name, minimum, maximum));
    return values;
  }
  if (!node.IsSequence() || node.size() != 4) {
    throw std::runtime_error(
            std::string("invalid steering parameter ") + name +
            ": expected null, integer or four-element array");
  }
  for (std::size_t i = 0; i < 4; ++i) {
    values[i] = steering_optional_raw_integer(node[i], name, minimum, maximum);
  }
  return values;
}

template<typename T>
T steering_module_value_or(
  const YAML::Node & module, const char * name, const T & fallback, std::size_t index)
{
  const auto value = module[name];
  if (!value) {
    return fallback;
  }
  try {
    return value.as<T>();
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "invalid swerve module[" + std::to_string(index) + "] parameter " + name + ": " +
            error.what());
  }
}

inline std::array<SwerveModuleConfig, 4> load_swerve_modules(
  const YAML::Node & parameters, const SteeringConfigFile & config)
{
  const auto node = parameters["modules"];
  if (!node) {
    return resolved_modules(config);
  }
  if (!node.IsSequence() || node.size() != 4) {
    throw std::runtime_error("modules must contain four entries");
  }
  std::array<SwerveModuleConfig, 4> modules{};
  for (std::size_t i = 0; i < modules.size(); ++i) {
    const auto entry = node[i];
    if (!entry || !entry["x_m"] || !entry["y_m"]) {
      throw std::runtime_error("each swerve module requires x_m and y_m");
    }
    auto module = legacy_module_config(config, i);
    module.x_m = steering_module_value_or<double>(entry, "x_m", module.x_m, i);
    module.y_m = steering_module_value_or<double>(entry, "y_m", module.y_m, i);
    module.wheel_radius_m = steering_module_value_or<double>(
      entry, "wheel_radius_m", module.wheel_radius_m, i);
    module.steering_gear_ratio = steering_module_value_or<double>(
      entry, "steering_gear_ratio", module.steering_gear_ratio, i);
    module.steering_encoder_resolution = steering_module_value_or<std::uint32_t>(
      entry, "steering_encoder_resolution", module.steering_encoder_resolution, i);
    module.steering_sign = steering_module_value_or<int>(
      entry, "steering_sign", module.steering_sign, i);
    module.heading_offset_rad = steering_module_value_or<double>(
      entry, "heading_offset_rad", module.heading_offset_rad, i);
    module.steering_min_rad = steering_module_value_or<double>(
      entry, "steering_min_rad", module.steering_min_rad, i);
    module.steering_max_rad = steering_module_value_or<double>(
      entry, "steering_max_rad", module.steering_max_rad, i);
    module.soft_margin_rad = steering_module_value_or<double>(
      entry, "soft_margin_rad", module.soft_margin_rad, i);
    module.drive_gear_ratio = steering_module_value_or<double>(
      entry, "drive_gear_ratio", module.drive_gear_ratio, i);
    module.drive_encoder_resolution = steering_module_value_or<std::uint32_t>(
      entry, "drive_encoder_resolution", module.drive_encoder_resolution, i);
    module.steering_inverted = steering_module_value_or<bool>(
      entry, "steering_inverted", module.steering_inverted, i);
    module.drive_inverted = steering_module_value_or<bool>(
      entry, "drive_inverted", module.drive_inverted, i);
    module.drive_max_motor_rpm = steering_module_value_or<double>(
      entry, "drive_max_motor_rpm", module.drive_max_motor_rpm, i);
    module.steering_rate_limit_rad_s = steering_module_value_or<double>(
      entry, "steering_rate_limit_rad_s", module.steering_rate_limit_rad_s, i);
    module.drive_acceleration_m_s2 = steering_module_value_or<double>(
      entry, "drive_acceleration_m_s2", module.drive_acceleration_m_s2, i);
    module.drive_deceleration_m_s2 = steering_module_value_or<double>(
      entry, "drive_deceleration_m_s2", module.drive_deceleration_m_s2, i);
    modules[i] = module;
  }
  return modules;
}

inline SteeringConfigFile load_steering_config(const std::string & path)
{
  SteeringConfigFile config;
  config.source_path = path;
  try {
    const auto parameters = steering_parameters(YAML::LoadFile(path));
    // 旧配置可以省略模式或声明 swerve；拒绝将其他模式的指令解释为舵轮运动。
    if (steering_value_or<std::string>(parameters, "steering_mode", "swerve") != "swerve") {
      throw std::invalid_argument("only swerve steering is supported");
    }
    config.can_bus = steering_value<std::int64_t>(parameters, "can_bus");
    config.wheel_base_m = steering_value<double>(parameters, "wheel_base_m");
    config.track_width_m = steering_value<double>(parameters, "track_width_m");
    config.max_steering_angle_rad = steering_value<double>(parameters, "max_steering_angle_rad");
    config.steering_forward_offset_rad = steering_value_or<double>(
      parameters, "steering_forward_offset_rad", config.steering_forward_offset_rad);
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
    config.swerve_speed_epsilon_m_s = steering_value_or<double>(
      parameters, "swerve_speed_epsilon_m_s", config.swerve_speed_epsilon_m_s);
    config.swerve_branch_hysteresis_rad = steering_value_or<double>(
      parameters, "swerve_branch_hysteresis_rad", config.swerve_branch_hysteresis_rad);
    config.drive_alignment_policy = steering_value_or<std::string>(
      parameters, "drive_alignment_policy", config.drive_alignment_policy);
    config.drive_alignment_pause_rad = steering_value_or<double>(
      parameters, "drive_alignment_pause_rad", config.drive_alignment_pause_rad);
    config.command_yaw_acceleration_rad_s2 = steering_value_or<double>(
      parameters, "command_yaw_acceleration_rad_s2", config.command_yaw_acceleration_rad_s2);
    config.steering_velocity_loop_kp = steering_optional_raw_parameter(
      parameters, "steering_velocity_loop_kp", 1, 32767);
    config.steering_velocity_loop_ki = steering_optional_raw_parameter(
      parameters, "steering_velocity_loop_ki", 0, 1023);
    config.steering_velocity_feedback_filter = steering_optional_raw_parameter(
      parameters, "steering_velocity_feedback_filter", 0, 45);
    config.steering_position_loop_kp = steering_optional_raw_parameter(
      parameters, "steering_position_loop_kp", 0, 32767);
    config.steering_position_smoothing_filter = steering_optional_raw_parameter(
      parameters, "steering_position_smoothing_filter", 1, 255);
    config.drive_velocity_loop_kp = steering_optional_raw_parameter(
      parameters, "drive_velocity_loop_kp", 1, 32767);
    config.drive_velocity_loop_ki = steering_optional_raw_parameter(
      parameters, "drive_velocity_loop_ki", 0, 1023);
    config.drive_velocity_feedback_filter = steering_optional_raw_parameter(
      parameters, "drive_velocity_feedback_filter", 0, 45);
    config.drive_position_loop_kp = steering_optional_raw_parameter(
      parameters, "drive_position_loop_kp", 0, 32767);
    config.drive_position_smoothing_filter = steering_optional_raw_parameter(
      parameters, "drive_position_smoothing_filter", 1, 255);
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
      !std::isfinite(config.swerve_speed_epsilon_m_s) ||
      config.swerve_speed_epsilon_m_s <= 0.0 ||
      !std::isfinite(config.swerve_branch_hysteresis_rad) ||
      config.swerve_branch_hysteresis_rad < 0.0 ||
      (config.drive_alignment_policy != "none" && config.drive_alignment_policy != "pause_all" &&
      config.drive_alignment_policy != "cosine") ||
      !std::isfinite(config.drive_alignment_pause_rad) ||
      config.drive_alignment_pause_rad <= 0.0 ||
      config.drive_alignment_pause_rad <= config.drive_steering_tolerance_rad ||
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
    const auto drive_can_buses = steering_value_or<std::vector<int>>(
      parameters, "drive_can_buses", std::vector<int>{1, 1, 2, 2});
    const auto drive_node_ids = steering_value_or<std::vector<int>>(
      parameters, "drive_node_ids", std::vector<int>{1, 2, 3, 4});
    const auto drive_inverted = steering_value_or<std::vector<bool>>(
      parameters, "drive_inverted", std::vector<bool>{false, false, false, false});
    if (node_ids.size() != 4 || inverted.size() != 4) {
      throw std::runtime_error("node_ids and inverted must contain four values");
    }
    if (drive_can_buses.size() != 4 || drive_node_ids.size() != 4 ||
      drive_inverted.size() != 4)
    {
      throw std::runtime_error(
              "drive_can_buses, drive_node_ids and drive_inverted must contain four values");
    }
    for (std::size_t i = 0; i < 4; ++i) {
      if (node_ids[i] < 1 || node_ids[i] > 127) {
        throw std::runtime_error("invalid steering node id");
      }
      config.motor.node_ids[i] = static_cast<std::uint8_t>(node_ids[i]);
      config.motor.inverted[i] = inverted[i];
      if (drive_can_buses[i] < 0 || drive_can_buses[i] >= 7 ||
        drive_node_ids[i] < 1 || drive_node_ids[i] > 127)
      {
        throw std::runtime_error("invalid drive CAN bus or node id");
      }
      config.drive_can_buses[i] = static_cast<unsigned int>(drive_can_buses[i]);
      config.drive_node_ids[i] = static_cast<std::uint8_t>(drive_node_ids[i]);
      config.drive_inverted[i] = drive_inverted[i];
    }
    if (parameters["modules"]) {
      config.modules = load_swerve_modules(parameters, config);
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

inline double calibration_motor_search_rpm(const SteeringConfigFile & config, std::size_t index)
{
  if (index >= 4) {
    throw std::invalid_argument("invalid steering module index");
  }
  const auto modules = resolved_modules(config);
  const auto & module = modules[index];
  const double rpm = config.calibration_search_speed_rpm * module.steering_gear_ratio;
  if (!std::isfinite(config.calibration_search_speed_rpm) ||
    config.calibration_search_speed_rpm <= 0.0 ||
    !std::isfinite(module.steering_gear_ratio) || module.steering_gear_ratio <= 0.0 ||
    module.steering_encoder_resolution == 0 ||
    !std::isfinite(config.calibration_speed_rpm) || config.calibration_speed_rpm <= 0.0 ||
    !std::isfinite(rpm) || rpm >= config.calibration_speed_rpm)
  {
    throw std::invalid_argument(
            "calibration_search_speed_rpm must be positive output RPM; "
            "search RPM * steering_gear_ratio must be below calibration_speed_rpm (motor RPM)");
  }
  return rpm;
}

}  // namespace chassis
