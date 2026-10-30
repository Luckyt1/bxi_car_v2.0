#pragma once

#include "arm/control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

#include <chrono>

namespace chassis
{

// A position jog only remains active while fresh held-button commands arrive.
class ArmPositionRemote
{
public:
  using Clock = std::chrono::steady_clock;

  ArmPositionRemote(rclcpp::Node & node, ArmController & arm)
  : node_(node), arm_(arm)
  {
    if (!arm.has_fourth_axis()) {return;}
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    const auto topic = node.declare_parameter<std::string>(
      "arm_position_topic", "/arm_motor4_jog", descriptor);
    subscription_ = node.create_subscription<std_msgs::msg::Int8>(
      topic, rclcpp::QoS(1).lifespan(rclcpp::Duration::from_seconds(0.1)),
      [this](std_msgs::msg::Int8::ConstSharedPtr message) {
        const auto now = Clock::now();
        // A delayed executor must not renew an already expired held command.
        update(now);
        const int direction = message->data;
        if (direction < -1 || direction > 1) {
          arm_.set_fourth_axis_jog(0);
          active_ = false;
          ready_ = false;
          RCLCPP_WARN(node_.get_logger(), "ARM_POSITION_REMOTE invalid command; halted");
          return;
        }
        if (direction == 0) {
          arm_.set_fourth_axis_jog(0);
          active_ = false;
          ready_ = true;
          return;
        }
        if (!ready_) {return;}
        arm_.set_fourth_axis_jog(direction);
        active_ = true;
        deadline_ = now + std::chrono::milliseconds(300);
      });
  }

  // Call before ArmController::update so an expired command cannot advance a handshake.
  void update(Clock::time_point now = Clock::now())
  {
    if (active_ && now >= deadline_) {
      arm_.set_fourth_axis_jog(0);
      active_ = false;
      ready_ = false;
      RCLCPP_WARN(
        node_.get_logger(), "ARM_POSITION_REMOTE timeout: halted after 300 ms; release to rearm");
    }
  }

private:
  rclcpp::Node & node_;
  ArmController & arm_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_;
  bool active_{false};
  bool ready_{false};
  Clock::time_point deadline_{};
};

}  // namespace chassis
