#include "example/app_interface.h"
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
using namespace std::chrono_literals;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

void targets_and_manifest()
{
  const arm::Pose pose{0.2, 0.3, 0.4};
  for (int code = 101; code <= 106; ++code) {
    const auto target = chassis::app_arm_target(pose, code);
    require(bool(target), "each Cartesian command must produce a target");
    const double dx = target->x - pose.x;
    const double dz = target->z - pose.z;
    const double dp = target->pitch - pose.pitch;
    const int sign = code % 2 ? 1 : -1;
    require(std::abs(dx - (code <= 102 ? sign * 0.01 : 0.0)) < 1e-12,
      "X steps must use metres");
    require(std::abs(dz - (code >= 103 && code <= 104 ? sign * 0.01 : 0.0)) < 1e-12,
      "Z steps must use metres");
    require(std::abs(dp - (code >= 105 ? sign * arm::kPi / 180.0 : 0.0)) < 1e-12,
      "pitch steps must use radians");
  }
  require(!chassis::app_arm_target(pose, 1), "legacy ELF3 values must never move the arm");
  require(!chassis::app_arm_target(pose, 109), "stop must not become a position target");
  for (const bool enabled : {false, true}) {
    for (const bool fourth : {false, true}) {
      const auto snapshot = YAML::Load(chassis::app_state_snapshot(enabled, fourth));
      const auto graph = snapshot["graph"];
      const auto expected = enabled ? (fourth ? 9U : 7U) : 0U;
      require(graph["remote_events"].IsMap(), "gateway requires object remote_events");
      require(graph["actions"].size() == expected, "only available hardware gets actions");
      require(graph["remote_events"].size() == expected, "all actions need a wire binding");
      for (const auto & action : graph["actions"]) {
        const auto event = action["event"].as<std::string>();
        require(graph["remote_events"][event]["slot"].as<std::string>() == "btn_10",
          "manifest actions must use the dedicated slot");
        require(action["from"].as<std::string>() == snapshot["current"]["name"].as<std::string>(),
          "App must be able to show actions for the current state");
      }
    }
  }
}

void ros_contract()
{
  auto node = std::make_shared<rclcpp::Node>(
    "car2_app_interface_test", rclcpp::NodeOptions().use_intra_process_comms(true));
  std::vector<int> commands;
  chassis::AppInterface interface(*node, true, true,
    [&](int command) {commands.push_back(command);});
  unsigned ready = 0;
  unsigned states = 0;
  auto ready_sub = node->create_subscription<std_msgs::msg::Header>(
    "/car2/ready", rclcpp::QoS(1).best_effort(),
    [&](std_msgs::msg::Header::ConstSharedPtr message) {
      require(message->frame_id == "car2", "readiness must identify car2");
      ++ready;
    });
  auto state_sub = node->create_subscription<std_msgs::msg::String>(
    "/car2/state_machine_info", rclcpp::QoS(1).best_effort(),
    [&](std_msgs::msg::String::ConstSharedPtr message) {
      require(YAML::Load(message->data)["graph"]["actions"].size() == 9,
        "full-arm state snapshot must traverse ROS unchanged");
      ++states;
    });
  auto publisher = node->create_publisher<std_msgs::msg::Int8>(
    "/car2/arm_command", rclcpp::QoS(1).lifespan(rclcpp::Duration::from_seconds(0.1)));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spin = [&] {executor.spin_some(); executor.spin_some();};
  spin();
  require(ready == 0 && states == 0, "constructing the interface must not announce hardware ready");
  for (int value : {0, 1, 100, 101, 107, 109, 110, -1}) {
    std_msgs::msg::Int8 message;
    message.data = static_cast<int8_t>(value);
    publisher->publish(message);
    spin();
  }
  require(commands == std::vector<int>({101, 107, 109}), "reject legacy and invalid arm commands");
  const auto now = chassis::AppInterface::Clock::now();
  interface.publish_ready(now);
  spin();
  require(ready == 1 && states == 1, "a successful control cycle publishes ready and state");
  interface.publish_ready(now + 20ms);
  spin();
  require(ready == 2 && states == 1, "ready follows control cycles; state is throttled");
  interface.publish_ready(now + 100ms);
  spin();
  require(ready == 3 && states == 2, "state refresh must keep the App manifest alive");
  spin();
  require(ready == 3, "no independent timer may advertise stalled hardware as alive");
  executor.remove_node(node);
}

void absent_axes_reject_commands()
{
  auto node = std::make_shared<rclcpp::Node>(
    "car2_app_three_axes_test", rclcpp::NodeOptions().use_intra_process_comms(true));
  std::vector<int> commands;
  chassis::AppInterface interface(*node, true, false,
    [&](int command) {commands.push_back(command);});
  auto publisher = node->create_publisher<std_msgs::msg::Int8>("/car2/arm_command", 1);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  for (int value : {107, 108, 109}) {
    std_msgs::msg::Int8 message;
    message.data = static_cast<int8_t>(value);
    publisher->publish(message);
    executor.spin_some();
    executor.spin_some();
  }
  require(commands == std::vector<int>({109}), "three-axis setups must retain stop without fourth-axis jog");
  executor.remove_node(node);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    targets_and_manifest();
    ros_contract();
    absent_axes_reject_commands();
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
