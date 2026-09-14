#pragma once

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace chassis
{

// Render the steering YAML with built-in Chinese notes. The YAML data is preserved:
// FL/RL/RR/FR means front-left, rear-left, rear-right, front-right; null tuning
// entries are read back and filled, numeric entries are sent to the driver and
// used again on the next start after the YAML is saved. CAN IDs are grouped by bus.
namespace steering_yaml_document_detail
{

struct FieldComment
{
  const char * name;
  const char * comment;
};

struct ParameterGroup
{
  const char * title;
  std::vector<FieldComment> fields;
};

inline std::string emit_yaml_node(const YAML::Node & node)
{
  YAML::Emitter emitter;
  emitter << node;
  if (!emitter.good()) {
    throw std::runtime_error("cannot render steering YAML: " + std::string(emitter.GetLastError()));
  }
  return emitter.c_str();
}

inline void append_lines(std::ostringstream & output, const std::string & text, int indent)
{
  std::istringstream input(text);
  std::string line;
  const std::string padding(static_cast<std::size_t>(indent), ' ');
  while (std::getline(input, line)) {
    output << padding << line << "\n";
  }
}

inline void append_mapping_entry(
  std::ostringstream & output,
  const YAML::Node & key,
  const YAML::Node & value,
  int indent)
{
  YAML::Node entry(YAML::NodeType::Map);
  entry.force_insert(key, value);
  append_lines(output, emit_yaml_node(entry), indent);
}

inline std::vector<ParameterGroup> parameter_groups()
{
  return {
    {"通信映射与 CANopen 反馈", {
        {"can_bus", "通信总线配置：CAN0 上挂四个转向轴；行走轴使用 drive_can_buses 指定。"},
        {"drive_can_buses", "四个行走轴 CAN 总线，顺序 FL 前左、RL 后左、RR 后右、FR 前右。"},
        {"node_ids", "四个转向轴 Node-ID，顺序 FL 前左、RL 后左、RR 后右、FR 前右，范围 1..127。"},
        {"drive_node_ids", "四个行走轴 Node-ID，顺序 FL 前左、RL 后左、RR 后右、FR 前右，范围 1..127。"},
        {"rpdo_transmission_type", "RPDO 传输类型，raw CANopen 值。"},
        {"tpdo_transmission_type", "TPDO 传输类型，raw CANopen 值。"},
        {"tpdo_inhibit_time", "TPDO 抑制时间，单位 0.1 ms。"},
        {"tpdo_event_timer_ms", "TPDO 事件周期，单位 ms。"},
      }},
    {"底盘尺寸与机械参数", {
        {"wheel_base_m", "底盘前后轮中心距离，单位 m。"},
        {"track_width_m", "底盘左右轮中心距离，单位 m。"},
        {"wheel_diameter_m", "行走轮直径，单位 m。"},
        {"encoder_resolution", "转向电机编码器每转计数。"},
        {"gear_ratio", "转向电机到舵轮输出轴减速比。"},
        {"drive_encoder_resolution", "行走电机编码器每转计数。"},
        {"drive_gear_ratio", "行走电机到车轮输出轴减速比。"},
        {"max_steering_angle_rad", "舵轮目标角度最大绝对值，单位 rad。"},
        {"steering_forward_offset_rad", "车体前进方向到转向零位的角度偏置，单位 rad。"},
        {"inverted", "四个转向轴方向反转，顺序 FL 前左、RL 后左、RR 后右、FR 前右。"},
        {"drive_inverted", "四个行走轴方向反转，顺序 FL 前左、RL 后左、RR 后右、FR 前右。"},
        {"zero_offset_inc", "四个转向轴校准中位编码器计数，顺序 FL 前左、RL 后左、RR 后右、FR 前右。"},
      }},
    {"运动控制", {
        {"command_timeout_s", "速度指令超时时间，单位 s。"},
        {"profile_velocity_rpm", "转向位置模式最大电机转速，单位 rpm。"},
        {"acceleration_rps2", "转向位置模式加速度，单位 rps^2。"},
        {"deceleration_rps2", "转向位置模式减速度，单位 rps^2。"},
        {"drive_max_output_rpm", "行走轮输出轴转速上限，单位 rpm。"},
        {"drive_acceleration_rpm_s", "行走轮输出轴加速度限制，单位 rpm/s。"},
        {"drive_deceleration_rpm_s", "行走轮输出轴减速度限制，单位 rpm/s。"},
        {"drive_steering_tolerance_rad", "转向误差容差，供对齐/回中判定；具体行走限制由 drive_alignment_policy 决定。"},
        {"steering_limit_margin_rad", "连续转向时避开软限位的角度余量，单位 rad。"},
        {"steering_rate_limit_rad_s", "转向目标角速度限制，单位 rad/s。"},
        {"drive_alignment_stop_rad",
          "旧控制器对齐停车阈值，单位 rad；Swerve 使用 drive_alignment_policy 和 drive_alignment_pause_rad。"},
        {"swerve_speed_epsilon_m_s", "Swerve 低速判定阈值，单位 m/s。"},
        {"swerve_branch_hysteresis_rad", "Swerve 角度分支切换迟滞，单位 rad。"},
        {"drive_alignment_policy", "转向未对齐时行走输出策略：none 不限制，cosine 按转角减速，pause_all 未对齐先停。"},
        {"drive_alignment_pause_rad", "pause_all 策略的转向误差暂停阈值，单位 rad。"},
        {"command_yaw_acceleration_rad_s2", "遥控旋转速度变化率限制，单位 rad/s^2。"},
      }},
    {"转向内部参数", {
        {"steering_velocity_loop_kp", "转向 0x60F9:01 速度环 Kp，raw，组 0，范围 1..32767；数组顺序 FL/RL/RR/FR。"},
        {"steering_velocity_loop_ki",
          "转向 0x60F9:02 速度环 Ki，raw，组 0，范围 0..1023；Ki=0 不代表全部积分关闭，另有 0x60F9:07。"},
        {"steering_velocity_feedback_filter",
          "转向 0x60F9:05 速度反馈滤波带宽 raw*20+100 Hz（非 CAN 读取频率），范围 0..45。"},
        {"steering_position_loop_kp", "转向 0x60FB:01 位置环 Kp，raw，对应 raw*0.01 Hz，组 0，范围 0..32767。"},
        {"steering_position_smoothing_filter", "转向 0x60FB:05 位置平滑滤波 raw，范围 1..255；越大越平滑且延迟增大。"},
      }},
    {"行走内部参数", {
        {"drive_velocity_loop_kp", "行走 0x60F9:01 速度环 Kp，raw，组 0，范围 1..32767；数组顺序 FL/RL/RR/FR。"},
        {"drive_velocity_loop_ki",
          "行走 0x60F9:02 速度环 Ki，raw，组 0，范围 0..1023；Ki=0 不代表全部积分关闭，另有 0x60F9:07。"},
        {"drive_velocity_feedback_filter",
          "行走 0x60F9:05 速度反馈滤波带宽 raw*20+100 Hz（非 CAN 读取频率），范围 0..45。"},
        {"drive_position_loop_kp", "行走 0x60FB:01 位置环 Kp，raw，对应 raw*0.01 Hz，组 0，范围 0..32767。"},
        {"drive_position_smoothing_filter", "行走 0x60FB:05 位置平滑滤波 raw，范围 1..255；越大越平滑且延迟增大。"},
      }},
    {"校准", {
        {"steering_calibrated", "四轮转向校准完成标记。"},
        {"calibration_speed_rpm", "校准电机轴转速保护上限，单位 rpm。"},
        {"calibration_search_speed_rpm", "校准搜索目标速度，舵轮输出轴单位 rpm。"},
        {"calibration_torque_percent", "兼容保留字段，按原值保存。"},
        {"calibration_torque_limit_nm", "校准估算输出力矩安全上限，单位 N*m。"},
        {"calibration_contact_torque_nm", "校准限位接触判定输出力矩，单位 N*m。"},
        {"calibration_min_limit_separation_rad", "两侧限位最小角距离，单位 rad。"},
        {"calibration_position_epsilon_inc", "回中位置允许编码器误差，单位 inc。"},
        {"calibration_stall_time_s", "兼容保留字段，按原值保存。"},
        {"calibration_current_threshold_a", "兼容保留字段，按原值保存，单位 A。"},
        {"calibration_current_rise_a", "兼容保留字段，按原值保存，单位 A。"},
        {"calibration_timeout_s", "每个校准阶段搜索超时，单位 s。"},
        {"calibration_node_id", "兼容保留字段，按原值保存。"},
        {"calibration_ignore_drive_fault", "兼容开关，必须保持 false；true 会被配置校验拒绝。"},
        {"motor_peak_current_a", "转向电机最大电流，单位 Arms。"},
        {"motor_torque_constant_nm_per_a", "转向电机力矩常数，单位 N*m/Arms。"},
      }},
    {"模块逐轮配置", {
        {"modules", "四个模块逐轮配置，顺序 FL/RL/RR/FR；字段含 x/y 位置 m、轮半径 m、转向/行走减速比、编码器、方向、软限位、速率和加减速度。"},
      }},
  };
}

inline bool contains_field(const std::vector<ParameterGroup> & groups, const std::string & name)
{
  for (const auto & group : groups) {
    const auto found = std::any_of(
      group.fields.begin(), group.fields.end(),
      [&name](const FieldComment & field) {return name == field.name;});
    if (found) {
      return true;
    }
  }
  return false;
}

inline void append_field(
  std::ostringstream & output,
  const YAML::Node & parameters,
  const FieldComment & field,
  int indent)
{
  const auto value = parameters[field.name];
  if (!value) {
    return;
  }
  const std::string padding(static_cast<std::size_t>(indent), ' ');
  output << padding << "# " << field.comment << "\n";
  append_mapping_entry(output, YAML::Node(field.name), value, indent);
}

inline void append_group(
  std::ostringstream & output,
  const YAML::Node & parameters,
  const ParameterGroup & group,
  int indent)
{
  bool has_any = false;
  for (const auto & field : group.fields) {
    has_any = has_any || static_cast<bool>(parameters[field.name]);
  }
  if (!has_any) {
    return;
  }

  const std::string padding(static_cast<std::size_t>(indent), ' ');
  output << "\n" << padding << "# ==================== " << group.title <<
    " ====================\n";
  if (std::string(group.title) == "转向内部参数" || std::string(group.title) == "行走内部参数") {
    output << padding << "# null 表示启动时读取驱动器实际值并回填 YAML；"
           << "数值会写入驱动器，下次启动继续使用。\n";
    output << padding << "# 调参数组顺序：FL 前左、RL 后左、RR 后右、FR 前右。\n";
  }
  for (const auto & field : group.fields) {
    append_field(output, parameters, field, indent);
  }
}

inline void append_ros_parameters(std::ostringstream & output, const YAML::Node & parameters)
{
  if (!parameters || parameters.size() == 0) {
    output << "  ros__parameters: {}\n";
    return;
  }

  output << "  # 内置备注在自动保存时重新生成。\n";
  output << "  ros__parameters:\n";
  const int indent = 4;
  const auto groups = parameter_groups();
  for (const auto & group : groups) {
    append_group(output, parameters, group, indent);
  }

  bool has_unknown = false;
  for (const auto entry : parameters) {
    has_unknown = has_unknown || !contains_field(groups, entry.first.as<std::string>());
  }
  if (!has_unknown) {
    return;
  }
  output << "\n    # ==================== 未分组保留参数 ====================\n";
  for (const auto entry : parameters) {
    if (!contains_field(groups, entry.first.as<std::string>())) {
      append_mapping_entry(output, entry.first, entry.second, indent);
    }
  }
}

inline void append_chassis_node(std::ostringstream & output, const YAML::Node & chassis)
{
  if (!chassis || chassis.size() == 0) {
    output << "chassis: {}\n";
    return;
  }

  output << "chassis:\n";
  for (const auto entry : chassis) {
    const auto key = entry.first.as<std::string>();
    if (key == "ros__parameters" && entry.second && entry.second.IsMap()) {
      append_ros_parameters(output, entry.second);
    } else {
      append_mapping_entry(output, entry.first, entry.second, 2);
    }
  }
}

}  // namespace steering_yaml_document_detail

inline std::string render_steering_yaml(const YAML::Node & document)
{
  if (!document.IsMap()) {
    return steering_yaml_document_detail::emit_yaml_node(document) + "\n";
  }
  if (document.size() == 0) {
    return "{}\n";
  }

  std::ostringstream output;
  for (const auto entry : document) {
    const auto key = entry.first.as<std::string>();
    if (key == "chassis" && entry.second && entry.second.IsMap()) {
      steering_yaml_document_detail::append_chassis_node(output, entry.second);
    } else {
      steering_yaml_document_detail::append_mapping_entry(output, entry.first, entry.second, 0);
    }
  }
  return output.str();
}

}  // namespace chassis
