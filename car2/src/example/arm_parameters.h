#pragma once

#include "arm/control.h"
#include "rclcpp/rclcpp.hpp"

#include <limits>
#include <vector>

namespace chassis
{
inline constexpr auto kYiyouCurrentParameter = "yiyou_motor_4_current_permille";
inline constexpr auto kYiyouPoseParameter = "yiyou_arm_target";

inline ArmMotionConfig declare_arm_motion_parameters(rclcpp::Node & node)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  ArmMotionConfig config;
  const auto path = node.declare_parameter<std::string>(
    "yiyou_arm_kinematics_file", "", descriptor);
  config.kinematics = load_arm_kinematics(path);
  const auto directions = node.declare_parameter<std::vector<std::int64_t>>(
    "yiyou_arm_motor_directions", {1, 1, 1}, descriptor);
  const auto zeros = node.declare_parameter<std::vector<std::int64_t>>(
    "yiyou_arm_zero_counts", {0, 0, 0}, descriptor);
  if (directions.size() != 3 || zeros.size() != 3) {
    throw std::invalid_argument("arm motor directions and zero counts must each have three entries");
  }
  for (std::size_t i = 0; i < 3; ++i) {
    if ((directions[i] != -1 && directions[i] != 1) ||
      zeros[i] < std::numeric_limits<std::int32_t>::min() ||
      zeros[i] > std::numeric_limits<std::int32_t>::max())
    {
      throw std::invalid_argument("arm directions must be +/-1 and zero counts must fit int32");
    }
    config.motor_directions[i] = static_cast<int>(directions[i]);
    config.zero_counts[i] = static_cast<std::int32_t>(zeros[i]);
  }
  config.speed_rpm = node.declare_parameter<double>(
    "yiyou_arm_speed_rpm", config.speed_rpm, descriptor);
  config.acceleration_rpm_s = node.declare_parameter<double>(
    "yiyou_arm_acceleration_rpm_s", 1.0, descriptor);
  config.home_on_start = node.declare_parameter<bool>(
    "yiyou_arm_home_on_start", true, descriptor);
  validate_arm_motion_config(config);
  return config;
}

inline void declare_arm_pose_parameter(rclcpp::Node & node)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description =
    "Runtime arm target [x_m, z_m, planar_pitch_rad] in the shoulder frame; startup must be empty";
  if (!node.declare_parameter<std::vector<double>>(
      kYiyouPoseParameter, std::vector<double>{}, descriptor).empty())
  {
    throw std::invalid_argument("yiyou_arm_target must be empty at startup; use runtime targets");
  }
}

inline rcl_interfaces::msg::SetParametersResult validate_arm_pose_parameters(
  const ArmController & arm, const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() != kYiyouPoseParameter) {continue;}
    try {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY ||
        parameter.as_double_array().size() != 3)
      {
        throw std::invalid_argument("yiyou_arm_target requires three doubles [x_m,z_m,pitch_rad]");
      }
      const auto & values = parameter.as_double_array();
      (void)arm.validate_target_pose({values[0], values[1], values[2]});
    } catch (const std::exception & error) {
      result.successful = false;
      result.reason = error.what();
      return result;
    }
  }
  return result;
}

// Apply only committed atomic parameter changes, never from a validation callback.
inline void apply_arm_pose_parameter(
  rclcpp::Node & node, ArmController & arm, std::vector<double> & last_request)
{
  const auto values = node.get_parameter(kYiyouPoseParameter).as_double_array();
  if (values == last_request) {return;}
  last_request = values;
  if (values.size() != 3) {throw std::invalid_argument("invalid committed arm target");}
  arm.set_target_pose({values[0], values[1], values[2]});
}

inline void declare_arm_current_parameter(rclcpp::Node & node, bool has_current_axis)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description =
    "Fourth arm axis CST target, signed rated-current permille; startup must be zero";
  descriptor.read_only = !has_current_axis;
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = -1000;
  range.to_value = 1000;
  range.step = 1;
  descriptor.integer_range.push_back(range);
  if (node.declare_parameter<std::int64_t>(kYiyouCurrentParameter, 0, descriptor) != 0) {
    throw std::invalid_argument("yiyou_motor_4_current_permille must be zero at startup");
  }
}

// Read only committed ROS values in the serial control timer. Rejected atomic
// parameter batches cannot change hardware through a validation callback.
inline void apply_arm_current_parameter(rclcpp::Node & node, ArmController & arm)
{
  if (!arm.has_current_axis()) {return;}
  const auto current = node.get_parameter(kYiyouCurrentParameter).as_int();
  if (current != arm.target_current_permille()) {arm.set_current_permille(current);}
}
}  // namespace chassis
