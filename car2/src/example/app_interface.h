#pragma once

#include "arm/kinematics.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

#include <array>
#include <chrono>
#include <functional>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

namespace chassis
{
// Dedicated values in the App's existing btn_10 slot; ELF3 button values have
// no meaning here. Movement pulses become single commands; safety stops repeat
// while gated so a delayed executor still receives a fresh stop request.
struct AppArmAction
{
  int code;
  const char * name;
  const char * label;
};

inline constexpr std::array<AppArmAction, 9> kAppArmActions{{
  {101, "arm_x_plus", "末端前移 1 cm"},
  {102, "arm_x_minus", "末端后移 1 cm"},
  {103, "arm_z_plus", "末端上移 1 cm"},
  {104, "arm_z_minus", "末端下移 1 cm"},
  {105, "arm_pitch_plus", "末端俯仰 +1°"},
  {106, "arm_pitch_minus", "末端俯仰 -1°"},
  {107, "arm_fourth_plus", "第四轴正向点动 100 ms"},
  {108, "arm_fourth_minus", "第四轴反向点动 100 ms"},
  {109, "arm_hold", "机械臂停止并保持"},
}};

inline std::optional<arm::Pose> app_arm_target(arm::Pose pose, int command)
{
  switch (command) {
    case 101: pose.x += 0.01; break;
    case 102: pose.x -= 0.01; break;
    case 103: pose.z += 0.01; break;
    case 104: pose.z -= 0.01; break;
    case 105: pose.pitch += arm::kPi / 180.0; break;
    case 106: pose.pitch -= arm::kPi / 180.0; break;
    default: return std::nullopt;
  }
  return pose;
}

// Matches the existing gateway StateMachineRelay schema. Actions stay in the
// current state; they are not fictitious locomotion modes or navigation goals.
inline std::string app_state_snapshot(bool arm_enabled, bool fourth_axis)
{
  std::ostringstream json;
  json << R"({"mode":"state","initial_state":"car2_ready","current":{"name":"car2_ready","id":0,"elapsed":0},"in_transition":false,"transition":null,"pending":null,"events":[],"graph":{"states":[{"name":"car2_ready","id":0,"label":"Car2 就绪"}],"transitions":[],"transition_profiles":{},"remote_events":{)";
  bool first = true;
  for (const auto & action : kAppArmActions) {
    if (!arm_enabled || (!fourth_axis && (action.code == 107 || action.code == 108))) {continue;}
    if (!first) {json << ',';}
    first = false;
    json << '"' << action.name << R"(_event":{"slot":"btn_10","value":)"
         << action.code << '}';
  }
  json << R"(},"actions":[)";
  first = true;
  for (const auto & action : kAppArmActions) {
    if (!arm_enabled || (!fourth_axis && (action.code == 107 || action.code == 108))) {continue;}
    if (!first) {json << ',';}
    first = false;
    json << R"({"from":"car2_ready","action":")" << action.name
         << R"(","event":")" << action.name << R"(_event","label":")"
         << action.label << R"("})";
  }
  json << "]}}";
  return json.str();
}

class AppInterface
{
public:
  using Clock = std::chrono::steady_clock;

  AppInterface(
    rclcpp::Node & node, bool arm_enabled, bool fourth_axis,
    std::function<void(int)> on_arm_command)
  : node_(node), snapshot_(app_state_snapshot(arm_enabled, fourth_axis))
  {
    ready_ = node.create_publisher<std_msgs::msg::Header>(
      "/car2/ready", rclcpp::QoS(1).best_effort());
    state_ = node.create_publisher<std_msgs::msg::String>(
      "/car2/state_machine_info", rclcpp::QoS(1).best_effort());
    if (arm_enabled) {
      commands_ = node.create_subscription<std_msgs::msg::Int8>(
        "/car2/arm_command", rclcpp::QoS(1).lifespan(rclcpp::Duration::from_seconds(0.1)),
        [on_arm_command = std::move(on_arm_command), fourth_axis](
          std_msgs::msg::Int8::ConstSharedPtr message) {
          const int command = message->data;
          if (command < 101 || command > 109 ||
            (!fourth_axis && (command == 107 || command == 108))) {return;}
          on_arm_command(command);
        });
    }
  }

  // Called by the hardware owner's control cycle AFTER successful feedback and
  // updates. No independent timer may keep a stalled/faulted controller alive.
  void publish_ready(Clock::time_point now = Clock::now())
  {
    std_msgs::msg::Header heartbeat;
    heartbeat.stamp = node_.now();
    heartbeat.frame_id = "car2";
    ready_->publish(heartbeat);
    if (now >= next_state_) {
      std_msgs::msg::String state;
      state.data = snapshot_;
      state_->publish(state);
      next_state_ = now + std::chrono::milliseconds(100);
    }
  }

private:
  rclcpp::Node & node_;
  std::string snapshot_;
  Clock::time_point next_state_{};
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr ready_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr commands_;
};
}  // namespace chassis
