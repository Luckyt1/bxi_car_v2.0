#pragma once

#include "example/arm_parameters.h"
#include "rcl_interfaces/msg/parameter.hpp"

#include <chrono>
#include <stdexcept>

namespace chassis
{

// A held-button command has a short lease; direct ROS parameter commands remain persistent.
class ArmCurrentRemote
{
public:
  using Clock = std::chrono::steady_clock;

  ArmCurrentRemote(rclcpp::Node & node, bool has_current_axis)
  : node_(node)
  {
    if (!has_current_axis) {return;}
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    const auto topic = node.declare_parameter<std::string>(
      "arm_current_topic", "/arm_motor4_current", descriptor);
    subscription_ = node.create_subscription<rcl_interfaces::msg::Parameter>(
      topic, rclcpp::QoS(1).lifespan(rclcpp::Duration::from_seconds(0.1)),
      [this](rcl_interfaces::msg::Parameter::ConstSharedPtr message) {
        if (message->name != kYiyouCurrentParameter ||
        message->value.type != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER ||
        message->value.integer_value < -1000 || message->value.integer_value > 1000)
        {
          if (active_) {commit(0);}
          RCLCPP_WARN(node_.get_logger(), "ARM_CURRENT_REMOTE invalid command discarded");
          return;
        }
        commit(message->value.integer_value);
      });
  }

  void update(Clock::time_point now = Clock::now())
  {
    if (active_ && now >= deadline_) {
      commit(0);
      RCLCPP_WARN(node_.get_logger(), "ARM_CURRENT_REMOTE timeout: current cleared after 300 ms");
    }
  }

private:
  void commit(std::int64_t current)
  {
    const auto result = node_.set_parameter(rclcpp::Parameter(kYiyouCurrentParameter, current));
    if (!result.successful) {
      throw std::runtime_error("remote current rejected: " + result.reason);
    }
    active_ = current != 0;
    deadline_ = Clock::now() + std::chrono::milliseconds(300);
  }

  rclcpp::Node & node_;
  rclcpp::Subscription<rcl_interfaces::msg::Parameter>::SharedPtr subscription_;
  bool active_{false};
  Clock::time_point deadline_{};
};

}  // namespace chassis
