#include "chassis/chassis_controller.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

class ChassisNode final : public rclcpp::Node
{
public:
  ChassisNode()
  : Node("chassis")
  {
    const auto path = declare_parameter<std::string>("steering_config_file", "");
    if (path.empty()) {
      throw std::invalid_argument("set steering_config_file to a writable calibration YAML file");
    }
    const double forward_rpm = declare_parameter<double>("post_calibration_rpm", 0.0);
    auto config = chassis::load_steering_config(path);
    if (!std::isfinite(forward_rpm) || forward_rpm < 0.0 ||
      forward_rpm > config.drive_max_output_rpm)
    {
      throw std::invalid_argument("post_calibration_rpm must be in [0, drive_max_output_rpm]");
    }
    config.steering_calibrated = false;
    chassis::save_steering_config(path, config);
    controller_ = std::make_unique<chassis::ChassisController>(config);
    const auto keep_running = [] {return rclcpp::ok();};
    controller_->initialize(keep_running);
    RCLCPP_INFO(
      get_logger(),
      "CAN0: calibrating steering axes in pairs [1,2] then [3,4]; travel axes inhibited");
    controller_->calibrate(keep_running);
    chassis::save_steering_config(path, controller_->config());
    if (!rclcpp::ok()) {throw std::runtime_error("startup cancelled");}
    if (forward_rpm > 0.0) {controller_->forward(forward_rpm);}
    if (forward_rpm > 0.0) {
      RCLCPP_INFO(
        get_logger(),
        "calibration complete; CAN1/2 ISWV travel forward %.3f output RPM; waiting for /cmd_vel_car",
        forward_rpm);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "calibration complete; travel wheels stopped; waiting for /cmd_vel_car");
    }
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_car", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        try {
          controller_->set_velocity(message->linear.x, message->linear.y, message->angular.z);
        } catch (const std::exception & error) {on_fault(error);}
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), [this] {
        try {controller_->update();} catch (const std::exception & error) {on_fault(error);}
      });
  }

private:
  void on_fault(const std::exception & error)
  {
    RCLCPP_ERROR(get_logger(), "chassis stopped: %s", error.what());
    if (timer_) {timer_->cancel();}
    rclcpp::shutdown();
  }

  std::unique_ptr<chassis::ChassisController> controller_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    // Single-threaded executor serializes the controller API calls.
    rclcpp::spin(std::make_shared<ChassisNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("chassis"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
