#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <linux/joystick.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class RemoteCtrl : public rclcpp::Node
{
public:
  RemoteCtrl()
  : Node("remote_ctrl"),
    publisher_(create_publisher<geometry_msgs::msg::Twist>(
        declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_car"), 10)),
    device_path_(declare_parameter<std::string>("device_path", "/dev/input/js0"))
  {
    const std::string cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();

    axis_angular_ = declare_parameter<int>("axis_angular", 6);
    axis_linear_ = declare_parameter<int>("axis_linear", 3);

    scale_linear_ = declare_parameter<double>("scale_linear", 0.6);
    scale_angular_ = declare_parameter<double>("scale_angular", 0.4);
    invert_linear_axis_ = declare_parameter<bool>("invert_linear_axis", true);
    invert_angular_axis_ = declare_parameter<bool>("invert_angular_axis", true);

    deadzone_ = declare_parameter<double>("deadzone", 0.05);
    timeout_sec_ = declare_parameter<double>("timeout_sec", 0.0);
    enable_button_ = declare_parameter<int>("enable_button", -1);
    stop_button_ = declare_parameter<int>("stop_button", 0);
    emergency_stop_button_ = declare_parameter<int>("emergency_stop_button", 11);

    max_accel_ = declare_parameter<double>("max_accel", 0.6);
    max_decel_ = declare_parameter<double>("max_decel", 1.2);
    if (!std::isfinite(scale_linear_) || scale_linear_ <= 0.0 ||
      !std::isfinite(scale_angular_) || scale_angular_ <= 0.0 ||
      !std::isfinite(deadzone_) || deadzone_ < 0.0 || deadzone_ >= 1.0 ||
      !std::isfinite(timeout_sec_) || timeout_sec_ < 0.0 ||
      !std::isfinite(max_accel_) || max_accel_ <= 0.0 ||
      !std::isfinite(max_decel_) || max_decel_ <= 0.0)
    {
      throw std::invalid_argument("invalid joystick scale, deadzone, timeout or acceleration");
    }
    open_joystick();
    timer_ =
      create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&RemoteCtrl::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Remote controller started");
    RCLCPP_INFO(get_logger(), "  device: %s", device_path_.c_str());
    RCLCPP_INFO(get_logger(), "  cmd topic: %s", cmd_vel_topic.c_str());
    RCLCPP_INFO(get_logger(), "  axes: linear=%d angular=%d", axis_linear_, axis_angular_);
    RCLCPP_INFO(
      get_logger(), "  scale: linear=%.4f m/s angular=%.4f rad/s",
      scale_linear_, scale_angular_);
    RCLCPP_INFO(
      get_logger(), "  direction: right-stick forward=forward, d-pad left=left turn");
    RCLCPP_INFO(
      get_logger(), "  axis inversion: linear=%s angular=%s",
      invert_linear_axis_ ? "true" : "false", invert_angular_axis_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(), "  buttons: stop=%d emergency-stop=%d",
      stop_button_, emergency_stop_button_);
    if (timeout_sec_ > 0.0) {
      RCLCPP_INFO(get_logger(), "  event timeout: %.2fs", timeout_sec_);
    } else {
      RCLCPP_INFO(get_logger(), "  event timeout: disabled");
    }
  }

  ~RemoteCtrl() override
  {
    if (joy_fd_ >= 0) {
      close(joy_fd_);
    }
  }

private:
  static double limit_rate(double current, double target, double max_accel, double max_decel)
  {
    double diff = target - current;

    if (diff > 0.0) {
      diff = std::min(diff, max_accel);
    } else {
      diff = std::max(diff, -max_decel);
    }

    return current + diff;
  }
  static double apply_deadzone(double value, double deadzone)
  {
    return std::abs(value) < deadzone ? 0.0 : value;
  }

  static double normalize_axis(int16_t value)
  {
    const double normalized = static_cast<double>(value) / 32767.0;
    return std::clamp(normalized, -1.0, 1.0);
  }

  bool button_pressed(int index) const
  {
    return index >= 0 && static_cast<size_t>(index) < buttons_.size() && buttons_[index] != 0;
  }

  double axis_value(int index) const
  {
    if (index < 0 || static_cast<size_t>(index) >= axes_.size()) {
      return 0.0;
    }
    return axes_[index];
  }
  static double smooth(double x)
  {
    constexpr double expo = 0.7;       // 0~1，越大越柔和
    return (1.0 - expo) * x + expo * x * x * x;
  }
  void open_joystick()
  {
    joy_fd_ = open(device_path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (joy_fd_ < 0) {
      RCLCPP_ERROR(
        get_logger(), "Failed to open %s: %s", device_path_.c_str(),
        std::strerror(errno));
      return;
    }

    unsigned char axis_count = 0;
    unsigned char button_count = 0;
    if (ioctl(joy_fd_, JSIOCGAXES, &axis_count) < 0) {
      RCLCPP_WARN(get_logger(), "Failed to query joystick axes: %s", std::strerror(errno));
      axis_count = 8;
    }
    if (ioctl(joy_fd_, JSIOCGBUTTONS, &button_count) < 0) {
      RCLCPP_WARN(get_logger(), "Failed to query joystick buttons: %s", std::strerror(errno));
      button_count = 16;
    }

    axes_.assign(axis_count, 0.0);
    buttons_.assign(button_count, 0);
    RCLCPP_INFO(
      get_logger(), "Opened joystick with %u axes and %u buttons", axis_count, button_count);
  }

  void read_joystick_events()
  {
    if (joy_fd_ < 0) {
      open_joystick();
      return;
    }

    js_event event;
    while (true) {
      const ssize_t bytes = read(joy_fd_, &event, sizeof(event));
      if (bytes == static_cast<ssize_t>(sizeof(event))) {
        joy_seen_ = true;
        last_joy_time_ = now();
        handle_event(event);
        if (emergency_stopped_) {break;}
        continue;
      }

      if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        break;
      }

      if (bytes == 0 || (bytes < 0 && errno == ENODEV)) {
        RCLCPP_WARN(get_logger(), "Joystick disconnected: %s", device_path_.c_str());
      } else if (bytes < 0) {
        RCLCPP_WARN(get_logger(), "Joystick read failed: %s", std::strerror(errno));
      }

      close(joy_fd_);
      joy_fd_ = -1;
      joy_seen_ = false;
      target_twist_ = geometry_msgs::msg::Twist();
      break;
    }
  }

  void handle_event(const js_event & event)
  {
    if (emergency_stopped_) {return;}
    const uint8_t type = event.type & ~JS_EVENT_INIT;
    if (type == JS_EVENT_AXIS && event.number < axes_.size()) {
      axes_[event.number] = normalize_axis(event.value);
    } else if (type == JS_EVENT_BUTTON && event.number < buttons_.size()) {
      buttons_[event.number] = event.value;
      if (static_cast<int>(event.number) == emergency_stop_button_ && event.value != 0) {
        emergency_stop();
        return;
      }
    }

    update_target_twist();
  }

  void emergency_stop()
  {
    emergency_stopped_ = true;
    target_twist_ = geometry_msgs::msg::Twist();
    smooth_twist_ = geometry_msgs::msg::Twist();
    publisher_->publish(smooth_twist_);
    RCLCPP_ERROR(
      get_logger(), "Emergency-stop button %d pressed: zero command sent; shutting down",
      emergency_stop_button_);
    rclcpp::shutdown();
  }

  void update_target_twist()
  {
    geometry_msgs::msg::Twist cmd;
    const bool enabled = enable_button_ < 0 || button_pressed(enable_button_);
    const bool stop = button_pressed(stop_button_);

    if (enabled && !stop) {

      double linear_x = apply_deadzone(axis_value(axis_linear_), deadzone_);
      double angular_z = apply_deadzone(axis_value(axis_angular_), deadzone_);
      if (invert_linear_axis_) {linear_x = -linear_x;}
      if (invert_angular_axis_) {angular_z = -angular_z;}


      cmd.linear.x = smooth(linear_x) * scale_linear_;
      cmd.angular.z = smooth(angular_z) * scale_angular_;

    }

    target_twist_ = cmd;
  }

  void timer_callback()
  {
    read_joystick_events();
    if (emergency_stopped_) {return;}

    geometry_msgs::msg::Twist cmd = target_twist_;
    if (!joy_seen_ ||
      (timeout_sec_ > 0.0 && (now() - last_joy_time_).seconds() > timeout_sec_))
    {
      cmd = geometry_msgs::msg::Twist();
    }

    const double dt = 0.05;

    smooth_twist_.linear.x = limit_rate(
      smooth_twist_.linear.x,
      cmd.linear.x,
      max_accel_ * dt,
      max_decel_ * dt
    );

    smooth_twist_.angular.z = cmd.angular.z;
    publisher_->publish(smooth_twist_);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string device_path_;
  int joy_fd_{-1};
  std::vector<double> axes_;
  std::vector<int> buttons_;

  geometry_msgs::msg::Twist target_twist_;
  geometry_msgs::msg::Twist smooth_twist_;
  rclcpp::Time last_joy_time_;
  bool joy_seen_{false};
  double max_accel_{0.6};
  double max_decel_{1.2};
  int axis_linear_{3};
  int axis_angular_{6};
  int enable_button_{-1};
  int stop_button_{0};
  int emergency_stop_button_{11};
  bool emergency_stopped_{false};
  bool invert_linear_axis_{true};
  bool invert_angular_axis_{true};
  double scale_linear_{0.6};
  double scale_angular_{0.4};
  double deadzone_{0.05};
  double timeout_sec_{0.5};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RemoteCtrl>());
  rclcpp::shutdown();
  return 0;
}
