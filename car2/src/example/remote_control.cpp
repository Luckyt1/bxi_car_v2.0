#include "example/remote_control.h"
#include "example/joystick_link_monitor.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <future>
#include <linux/joystick.h>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include "chassis/remote_kinematics.hpp"
#include "arm/joystick.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace chassis
{
namespace
{
YAML::Node load_remote_parameters(const std::string & path)
{
  if (path.empty()) {return YAML::Node(YAML::NodeType::Map);}
  try {
    const auto document = YAML::LoadFile(path);
    if (!document.IsMap()) {
      throw std::invalid_argument("expected a YAML mapping");
    }
    const auto remote = document["remote_ctrl"];
    if (!remote) {return YAML::Node(YAML::NodeType::Map);}
    if (!remote.IsMap() || !remote["ros__parameters"].IsMap()) {
      throw std::invalid_argument("remote_ctrl.ros__parameters must be a mapping");
    }
    return remote["ros__parameters"];
  } catch (const std::exception & error) {
    throw std::invalid_argument("remote configuration " + path + ": " + error.what());
  }
}

double remote_value_or(
  const YAML::Node & parameters, const char * name, double fallback, const std::string & path)
{
  const auto value = parameters[name];
  if (!value) {return fallback;}
  try {
    return value.as<double>();
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(
            "remote configuration parameter " + std::string(name) + " in " + path + ": " +
            error.what());
  }
}

class RemoteCtrl : public rclcpp::Node
{
public:
  RemoteCtrl(
    const rclcpp::NodeOptions & options, std::shared_ptr<RemoteControlSession> session,
    bool enable_arm_controls, const std::string & default_steering_config_file,
    std::shared_ptr<JoystickLinkMonitor> link_monitor)
  : Node("remote_ctrl", options),
    session_(std::move(session)),
    enable_arm_controls_(enable_arm_controls),
    publisher_(create_publisher<geometry_msgs::msg::Twist>(
        declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_car"),
        rclcpp::QoS(1).lifespan(rclcpp::Duration::from_seconds(0.3)))),
    device_path_(declare_parameter<std::string>("device_path", "/dev/input/js0"))
  {
    const std::string cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();

    axis_angular_ = declare_parameter<int>("axis_angular", 6);
    axis_linear_ = declare_parameter<int>("axis_linear", 3);
    axis_lateral_ = declare_parameter<int>("axis_lateral", 0);
    if (axis_linear_ < 0 || axis_lateral_ < 0 || axis_angular_ < -1 ||
      axis_linear_ == 7 || axis_lateral_ == 7 || axis_angular_ == 7)
    {
      throw std::invalid_argument("axis7 is reserved for the arm; axis_angular=-1 disables yaw");
    }
    if (enable_arm_controls_) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.read_only = true;
      invert_arm_direction_ = declare_parameter<bool>("invert_arm_direction", false, descriptor);
      arm_translation_step_m_ = declare_parameter<double>(
        "arm_translation_step_m", 0.01, descriptor);
      arm_pitch_step_rad_ = declare_parameter<double>(
        "arm_pitch_step_rad", std::acos(-1.0) / 180.0, descriptor);
      if (!std::isfinite(arm_translation_step_m_) || arm_translation_step_m_ <= 0.0 ||
        !std::isfinite(arm_pitch_step_rad_) || arm_pitch_step_rad_ <= 0.0)
      {
        throw std::invalid_argument("arm translation/pitch steps must be finite and positive");
      }
      const auto arm_node = declare_parameter<std::string>("arm_node", "/chassis", descriptor);
      arm_current_permille_ = declare_parameter<int>("arm_current_permille", 50, descriptor);
      if (arm_current_permille_ < 1 || arm_current_permille_ > 1000) {
        throw std::invalid_argument("arm_current_permille must be in [1,1000]");
      }
      arm_current_publisher_ = create_publisher<rcl_interfaces::msg::Parameter>(
        declare_parameter<std::string>("arm_current_topic", "/arm_motor4_current", descriptor),
        rclcpp::QoS(1).lifespan(rclcpp::Duration::from_seconds(0.1)));
      const int speech_domain = declare_parameter<int>("arm_speech_domain_id", -1, descriptor);
      if (speech_domain < -1 || speech_domain > 232) {
        throw std::invalid_argument("arm_speech_domain_id must be -1 or in [0,232]");
      }
      if (speech_domain == -1) {
        arm_speech_publisher_ = create_publisher<std_msgs::msg::String>("/tts/say", 10);
      } else {
        // Speech may share a domain with the robot gateway, separate from motor control.
        arm_speech_context_ = std::make_shared<rclcpp::Context>();
        rclcpp::InitOptions speech_init;
        speech_init.auto_initialize_logging(false);
        speech_init.set_domain_id(static_cast<std::size_t>(speech_domain));
        arm_speech_context_->init(0, nullptr, speech_init);
        auto speech_options = rclcpp::NodeOptions().context(arm_speech_context_)
          .use_global_arguments(false).start_parameter_services(false).enable_rosout(false);
        arm_speech_node_ = std::make_shared<rclcpp::Node>("arm_selection_speech", speech_options);
        arm_speech_publisher_ = arm_speech_node_->create_publisher<std_msgs::msg::String>(
          get_node_topics_interface()->resolve_topic_name("/tts/say"), 10);
      }
      arm_get_client_ = create_client<GetParameters>(arm_node + "/get_parameters");
      arm_set_client_ = create_client<SetParameters>(arm_node + "/set_parameters_atomically");
      arm_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        declare_parameter<std::string>("arm_pose_topic", "/arm_pose", descriptor), 1,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
          observe_arm_pose(*message);
        });
    }

    const auto config_path = declare_parameter<std::string>(
      "steering_config_file", default_steering_config_file);
    const auto parameters = load_remote_parameters(config_path);
    const auto configured_default = [&](const char * name, double fallback) {
        return remote_value_or(parameters, name, fallback, config_path);
      };
    remote_config_.scale_linear_m_s = declare_parameter<double>(
      "scale_linear", configured_default("scale_linear", 0.6));
    remote_config_.scale_lateral_m_s = declare_parameter<double>(
      "scale_lateral", configured_default("scale_lateral", 0.6));
    remote_config_.scale_angular_rad_s = declare_parameter<double>(
      "scale_angular", configured_default("scale_angular", 0.4));
    invert_linear_axis_ = declare_parameter<bool>("invert_linear_axis", true);
    invert_lateral_axis_ = declare_parameter<bool>("invert_lateral_axis", true);
    invert_angular_axis_ = declare_parameter<bool>("invert_angular_axis", true);

    deadzone_ = declare_parameter<double>("deadzone", 0.05);
    joystick_low_pass_time_constant_s_ =
      declare_parameter<double>("joystick_low_pass_time_constant_s", 0.1);
    timeout_sec_ = declare_parameter<double>("timeout_sec", 0.0);
    enable_button_ = declare_parameter<int>("enable_button", -1);
    stop_button_ = declare_parameter<int>("stop_button", enable_arm_controls_ ? -1 : 0);
    emergency_stop_button_ = declare_parameter<int>("emergency_stop_button", 11);
    rcl_interfaces::msg::ParameterDescriptor startup_descriptor;
    startup_descriptor.read_only = true;
    const bool start_chassis = declare_parameter<bool>(
      "start_chassis", session_ != nullptr, startup_descriptor);
    start_button_ = declare_parameter<int>("start_button", 14, startup_descriptor);
    if (enable_arm_controls_) {
      for (const int button : {enable_button_, stop_button_, emergency_stop_button_,
          start_chassis ? start_button_ : -1})
      {
        if (button == 0 || button == 4) {
          throw std::invalid_argument("buttons 0/4 are reserved for fourth-axis current");
        }
      }
    }
    if (start_chassis) {
      if (!session_) {throw std::invalid_argument("start_chassis requires a startup session");}
      if (start_button_ < 0 || start_button_ > 255 ||
        emergency_stop_button_ < 0 || emergency_stop_button_ > 255 ||
        start_button_ == emergency_stop_button_ || start_button_ == stop_button_ ||
        start_button_ == enable_button_ || start_button_ == 1 || start_button_ == 3)
      {
        throw std::invalid_argument("invalid or conflicting start/emergency-stop button");
      }
    }
    if (session_) {
      session_->start_chassis = start_chassis;
      session_->stage = start_chassis ? RemoteControlSession::Stage::waiting :
        RemoteControlSession::Stage::running;
    }
    for (const int button : {enable_button_, stop_button_, emergency_stop_button_}) {
      if (button == 1 || button == 3) {
        throw std::invalid_argument("buttons 3/1 are reserved for arm x/z/pitch selection");
      }
    }

    max_accel_ = declare_parameter<double>("max_accel", configured_default("max_accel", 1.2));
    max_decel_ = declare_parameter<double>("max_decel", configured_default("max_decel", 2.4));
    if (!std::isfinite(deadzone_) || deadzone_ < 0.0 || deadzone_ >= 1.0 ||
      !std::isfinite(joystick_low_pass_time_constant_s_) ||
      joystick_low_pass_time_constant_s_ < 0.0 ||
      !std::isfinite(timeout_sec_) || timeout_sec_ < 0.0 ||
      !std::isfinite(max_accel_) || max_accel_ <= 0.0 ||
      !std::isfinite(max_decel_) || max_decel_ <= 0.0)
    {
      throw std::invalid_argument(
              "invalid joystick deadzone, low-pass time constant, timeout or acceleration");
    }
    chassis::validate_remote_kinematics_config(remote_config_);
    link_monitor_ = link_monitor ? std::move(link_monitor) :
      std::make_shared<JoystickLinkMonitor>(device_path_);
    open_joystick();
    timer_ =
      create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&RemoteCtrl::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Remote controller started");
    RCLCPP_INFO(get_logger(), "  link watchdog: active HID query, 0.50 s timeout; fail closed");
    if (startup_pending()) {
      RCLCPP_INFO(
        get_logger(), "Waiting for button%d; chassis power and calibration have not started",
        start_button_);
    }
    RCLCPP_INFO(get_logger(), "  device: %s", device_path_.c_str());
    RCLCPP_INFO(get_logger(), "  cmd topic: %s", cmd_vel_topic.c_str());
    RCLCPP_INFO(
      get_logger(), "  axes: linear=%d lateral=%d angular=%d",
      axis_linear_, axis_lateral_, axis_angular_);
    RCLCPP_INFO(
      get_logger(), "  scale: linear=%.4f m/s lateral=%.4f m/s angular=%.4f rad/s",
      remote_config_.scale_linear_m_s, remote_config_.scale_lateral_m_s,
      remote_config_.scale_angular_rad_s);
    RCLCPP_INFO(
      get_logger(), "  joystick low-pass time constant: %.4f s (0 disables filtering)",
      joystick_low_pass_time_constant_s_);
    RCLCPP_INFO(
      get_logger(), "  mapping: axis %d=vx, axis %d=vy, axis %d=yaw",
      axis_linear_, axis_lateral_, axis_angular_);
    RCLCPP_INFO(
      get_logger(), "  axis inversion: linear=%s lateral=%s angular=%s",
      invert_linear_axis_ ? "true" : "false", invert_lateral_axis_ ? "true" : "false",
      invert_angular_axis_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(), "  buttons: stop=%d emergency-stop=%d",
      stop_button_, emergency_stop_button_);
    RCLCPP_INFO(get_logger(), "Waiting for active motion axes to report neutral");
    if (enable_arm_controls_) {
      RCLCPP_INFO(
        get_logger(), "Arm remote: button3 selects previous coordinate, button1 selects next; "
        "cycle x/z/pitch; axis7 negative=+step, positive=-step; hold repeats every 0.1 s; "
        "center stops repeat; center after coordinate/direction changes");
      RCLCPP_INFO(
        get_logger(), "Arm selected=x; translation step=%.4f m, pitch step=%.6f rad; "
        "direction inverted=%s; target node=%s",
        arm_translation_step_m_, arm_pitch_step_rad_, invert_arm_direction_ ? "true" : "false",
        get_parameter("arm_node").as_string().c_str());
      RCLCPP_INFO(
        get_logger(), "Arm current: hold button0=+%d, button4=-%d rated-current permille; "
        "release or both pressed=0; release both before use; button11 remains emergency stop",
        arm_current_permille_, arm_current_permille_);
    } else {
      RCLCPP_INFO(get_logger(), "Chassis-only remote: Cartesian arm controls disabled");
    }
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
  using GetParameters = rcl_interfaces::srv::GetParameters;
  using SetParameters = rcl_interfaces::srv::SetParametersAtomically;

  bool arm_busy() const {return arm_get_request_ || arm_set_request_;}

  bool startup_pending() const
  {
    return session_ && session_->stage != RemoteControlSession::Stage::running;
  }

  static const char * arm_coordinate_name(unsigned coordinate)
  {
    constexpr std::array<const char *, 3> names{"x", "z", "pitch"};
    return names.at(coordinate);
  }

  void observe_arm_pose(const geometry_msgs::msg::PoseStamped & message)
  {
    const auto & position = message.pose.position;
    const auto & q = message.pose.orientation;
    // The input is the controller's shoulder-frame planar FK, not an arbitrary ROS pose.
    if (message.header.frame_id != "arm_shoulder" ||
      !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
      std::abs(position.y) > 1e-6 || std::abs(q.x) > 1e-6 || std::abs(q.z) > 1e-6 ||
      std::abs(q.y * q.y + q.w * q.w - 1.0) > 1e-6)
    {
      arm_actual_pose_.reset();
      return;
    }
    arm_actual_pose_ = std::array<double, 3>{position.x, position.z, -2.0 * std::atan2(q.y, q.w)};
    arm_pose_received_ = std::chrono::steady_clock::now();
  }

  bool arm_feedback_fresh() const
  {
    return arm_actual_pose_ && std::chrono::steady_clock::now() - arm_pose_received_ <
           std::chrono::milliseconds(500);
  }

  void handle_arm_action(const chassis::ArmJoystickAction & action)
  {
    if (action.selection_changed) {
      arm_action_.reset();
      // 未提交的步进取消；已提交的请求继续等待结果，不向新坐标重放。
      if (arm_get_request_) {cancel_arm_adjustment("coordinate selection changed");}
      RCLCPP_INFO(
        get_logger(), "ARM_SELECTED coordinate=%s",
        arm_coordinate_name(action.coordinate));
      static constexpr std::array<const char *, 3> speech{
        "现在前后移动", "现在上下移动", "第三个关节移动"};
      std_msgs::msg::String message;
      message.data = speech.at(action.coordinate);
      arm_speech_publisher_->publish(message);
    }
    if (joystick_armed_ && action.direction != 0 && !arm_busy() && !arm_action_) {
      arm_action_ = action;
    }
  }

  void cancel_arm_adjustment(const char * reason)
  {
    if (arm_get_request_) {
      arm_get_client_->remove_pending_request(arm_get_request_->request_id);
      arm_get_request_.reset();
    }
    if (arm_set_request_) {
      RCLCPP_WARN(
        get_logger(), "Arm coordinate=%s: %s; submitted step may have taken effect",
        arm_coordinate_name(arm_requested_coordinate_), reason);
      arm_set_client_->remove_pending_request(arm_set_request_->request_id);
      arm_set_request_.reset();
    }
    arm_action_.reset();
  }

  void update_arm_adjustment()
  {
    const auto time = std::chrono::steady_clock::now();
    if (arm_busy() && time >= arm_deadline_) {
      RCLCPP_WARN(get_logger(), "Arm parameter service timeout; step discarded, no retry");
      arm_joystick_.suspend_repeat();
      cancel_arm_adjustment("timeout");
      return;
    }
    try {
      if (arm_set_request_) {
        if (arm_set_request_->wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
          return;
        }
        const auto response = arm_set_request_->get();
        arm_set_request_.reset();
        if (response->result.successful) {
          RCLCPP_INFO(
            get_logger(), "ARM_REMOTE coordinate=%s target=[%.6f,%.6f,%.6f] accepted",
            arm_coordinate_name(arm_requested_coordinate_), arm_requested_pose_[0],
            arm_requested_pose_[1], arm_requested_pose_[2]);
        } else {
          RCLCPP_WARN(
            get_logger(), "ARM_REMOTE coordinate=%s rejected: %s",
            arm_coordinate_name(arm_requested_coordinate_), response->result.reason.c_str());
          if (response->result.reason != "arm position handshake is pending") {
            arm_joystick_.suspend_repeat();
          }
        }
        return;
      }
      if (arm_get_request_) {
        if (arm_get_request_->wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
          return;
        }
        const auto response = arm_get_request_->get();
        arm_get_request_.reset();
        if (!arm_feedback_fresh()) {
          throw std::runtime_error("arm pose feedback is missing or stale; step discarded");
        }
        if (response->values.size() != 1 ||
          response->values[0].type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY)
        {
          throw std::runtime_error("Arm target parameter is missing or invalid");
        }
        const auto & target = response->values[0].double_array_value;
        if (target.empty()) {
          arm_requested_pose_ = *arm_actual_pose_;
        } else {
          if (target.size() != 3 ||
            !std::all_of(
              target.begin(), target.end(), [](double value) {
                return std::isfinite(value);
              }))
          {
            throw std::runtime_error("Arm target must contain three finite coordinates");
          }
          std::copy(target.begin(), target.end(), arm_requested_pose_.begin());
        }
        arm_requested_pose_[arm_requested_coordinate_] += arm_requested_delta_;
        if (!std::isfinite(arm_requested_pose_[arm_requested_coordinate_])) {
          throw std::runtime_error("arm step overflows the selected coordinate");
        }
        auto request = std::make_shared<SetParameters::Request>();
        request->parameters.push_back(
          rclcpp::Parameter(
            "yiyou_arm_target", std::vector<double>(
              arm_requested_pose_.begin(), arm_requested_pose_.end())).to_parameter_msg());
        arm_set_request_.emplace(arm_set_client_->async_send_request(request));
        return;
      }
      if (!arm_action_) {return;}
      const auto action = *arm_action_;
      arm_action_.reset();
      if (!arm_feedback_fresh()) {
        RCLCPP_WARN(get_logger(), "Arm pose feedback is missing or stale; step discarded");
        arm_joystick_.suspend_repeat();
        return;
      }
      if (!arm_get_client_->service_is_ready() || !arm_set_client_->service_is_ready()) {
        RCLCPP_WARN(get_logger(), "Arm parameter service unavailable; step discarded");
        arm_joystick_.suspend_repeat();
        return;
      }
      arm_requested_coordinate_ = action.coordinate;
      arm_requested_repeat_ = action.repeated;
      arm_requested_delta_ = action.direction * (invert_arm_direction_ ? -1 : 1) *
        (action.coordinate == 2 ? arm_pitch_step_rad_ : arm_translation_step_m_);
      auto request = std::make_shared<GetParameters::Request>();
      request->names = {"yiyou_arm_target"};
      arm_deadline_ = time + std::chrono::seconds(2);
      arm_get_request_.emplace(arm_get_client_->async_send_request(request));
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Arm remote request failed: %s", error.what());
      arm_joystick_.suspend_repeat();
      cancel_arm_adjustment("request failed");
    }
  }

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

  void update_arm_current(bool enabled)
  {
    if (!enable_arm_controls_) {return;}
    const bool positive = button_pressed(0);
    const bool negative = button_pressed(4);
    if (!enabled || !arm_feedback_fresh() || (positive && negative)) {
      arm_current_ready_ = false;
    } else if (arm_current_seen_[0] && arm_current_seen_[1] && !positive && !negative) {
      arm_current_ready_ = true;
    }
    const int target = arm_current_ready_ && enabled && positive != negative ?
      (positive ? arm_current_permille_ : -arm_current_permille_) : 0;
    if (target != 0 || last_arm_current_ != 0) {
      arm_current_publisher_->publish(
        rclcpp::Parameter("yiyou_motor_4_current_permille", target).to_parameter_msg());
      if (target != last_arm_current_) {
        RCLCPP_INFO(get_logger(), "ARM_CURRENT_REMOTE target_current_permille=%d", target);
      }
    }
    last_arm_current_ = target;
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
  void open_joystick()
  {
    const auto now = std::chrono::steady_clock::now();
    if (now < next_reconnect_) {return;}
    next_reconnect_ = now + std::chrono::seconds(1);
    joy_fd_ = open(device_path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (joy_fd_ < 0) {
      const int error = errno;
      RCLCPP_ERROR(
        get_logger(), "Failed to open %s: %s", device_path_.c_str(),
        std::strerror(error));
      if (startup_pending() && error != ENOENT && error != ENODEV && error != ENXIO &&
        error != EIO)
      {
        throw std::runtime_error("cannot open joystick: " + std::string(std::strerror(error)));
      }
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

    if (startup_pending() && std::max(start_button_, emergency_stop_button_) >= button_count) {
      close(joy_fd_);
      joy_fd_ = -1;
      throw std::invalid_argument("joystick does not provide the configured start/stop buttons");
    }

    start_released_ = false;
    emergency_stop_released_ = false;
    axes_.assign(axis_count, 0.0);
    axis_seen_.assign(axis_count, false);
    joystick_armed_ = false;
    arm_joystick_.reset();
    arm_current_ready_ = false;
    arm_current_seen_.fill(false);
    buttons_.assign(button_count, 0);
    RCLCPP_INFO(
      get_logger(), "Opened joystick with %u axes and %u buttons", axis_count, button_count);
    if (enable_arm_controls_) {
      RCLCPP_INFO(
        get_logger(),
        "ARM_SELECTED coordinate=x; waiting for axis7 neutral and buttons 3/1 released");
    }
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
        last_joy_time_ = std::chrono::steady_clock::now();
        handle_event(event);
        if (emergency_stopped_) {break;}
        continue;
      }

      if (bytes < 0 && errno == EINTR) {continue;}
      if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        break;
      }

      if (bytes == 0 || (bytes < 0 && errno == ENODEV)) {
        RCLCPP_WARN(get_logger(), "Joystick disconnected: %s", device_path_.c_str());
      } else if (bytes < 0) {
        RCLCPP_WARN(get_logger(), "Joystick read failed: %s", std::strerror(errno));
      }

      invalidate_joystick("joystick disconnected");
      break;
    }
  }

  void invalidate_joystick(const char * reason)
  {
    if (joy_fd_ >= 0) {close(joy_fd_);}
    joy_fd_ = -1;
    joy_seen_ = false;
    joystick_armed_ = false;
    start_released_ = false;
    emergency_stop_released_ = false;
    axes_.clear();
    axis_seen_.clear();
    buttons_.clear();
    if (startup_pending()) {
      if (session_->stage == RemoteControlSession::Stage::starting) {
        RCLCPP_ERROR(get_logger(), "Joystick link lost during startup; cancelling calibration");
        rclcpp::shutdown(get_node_base_interface()->get_context());
      } else {
        session_->stage = RemoteControlSession::Stage::waiting;
      }
    }
    arm_joystick_.reset();
    arm_current_seen_.fill(false);
    arm_current_ready_ = false;
    update_arm_current(false);
    cancel_arm_adjustment(reason);
    target_twist_ = geometry_msgs::msg::Twist();
  }

  void handle_event(const js_event & event)
  {
    if (emergency_stopped_) {return;}
    const uint8_t type = event.type & ~JS_EVENT_INIT;
    if (type == JS_EVENT_AXIS && event.number < axes_.size()) {
      axes_[event.number] = normalize_axis(event.value);
      axis_seen_[event.number] = true;
      if (event.number == 7 && (event.type & JS_EVENT_INIT) != 0) {
        cancel_arm_adjustment("joystick initialization");
      }
      if (enable_arm_controls_) {
        handle_arm_action(
          arm_joystick_.observe(
            event.number, axes_[event.number], (event.type & JS_EVENT_INIT) != 0));
        if (event.number == 7 && !arm_joystick_.repeat_active()) {
          if (arm_action_ && arm_action_->repeated) {arm_action_.reset();}
          if (arm_get_request_ && arm_requested_repeat_) {
            cancel_arm_adjustment("held arm step released");
          }
        }
      }
    } else if (type == JS_EVENT_BUTTON && event.number < buttons_.size()) {
      buttons_[event.number] = event.value;
      if (enable_arm_controls_ && (event.number == 0 || event.number == 4)) {
        arm_current_seen_[event.number == 0 ? 0 : 1] = true;
        if ((event.type & JS_EVENT_INIT) != 0 || (button_pressed(0) && button_pressed(4))) {
          update_arm_current(false);
        }
      }
      if ((static_cast<int>(event.number) == stop_button_ && event.value != 0) ||
        (static_cast<int>(event.number) == enable_button_ && event.value == 0))
      {
        update_arm_current(false);
        arm_joystick_.disarm();
        cancel_arm_adjustment("stop/enable button");
      }
      if (static_cast<int>(event.number) == emergency_stop_button_ && event.value != 0) {
        emergency_stop();
        return;
      }
      if (session_ && session_->stage == RemoteControlSession::Stage::waiting) {
        if (static_cast<int>(event.number) == emergency_stop_button_) {
          emergency_stop_released_ = event.value == 0;
        }
        if (static_cast<int>(event.number) == start_button_) {
          const bool was_released = start_released_;
          start_released_ = event.value == 0;
          if (event.value != 0 && was_released && emergency_stop_released_ &&
            (event.type & JS_EVENT_INIT) == 0)
          {
            session_->stage = RemoteControlSession::Stage::requested;
            RCLCPP_INFO(get_logger(), "button%d pressed: chassis startup requested", start_button_);
            if (arm_speech_publisher_) {
              std_msgs::msg::String message;
              message.data = "开始校准，请不要乱动";
              arm_speech_publisher_->publish(message);
              calibration_finish_pending_ = true;
            }
          }
        }
      }
      if ((event.number == 1 || event.number == 3) && (event.type & JS_EVENT_INIT) != 0) {
        cancel_arm_adjustment("joystick initialization");
      }
      if (enable_arm_controls_) {
        handle_arm_action(
          arm_joystick_.observe_button(
            event.number, event.value != 0, (event.type & JS_EVENT_INIT) != 0));
      }
    }

  }

  void emergency_stop()
  {
    emergency_stopped_ = true;
    update_arm_current(false);
    arm_joystick_.disarm();
    cancel_arm_adjustment("emergency stop");
    publish_zero_velocity();
    RCLCPP_ERROR(
      get_logger(), "Emergency-stop button %d pressed: zero command sent; shutting down",
      emergency_stop_button_);
    rclcpp::shutdown(get_node_base_interface()->get_context());
  }

  void publish_zero_velocity()
  {
    // A lost controller must bypass both input filtering and the command ramp.
    filtered_axes_.fill(0.0);
    target_twist_ = geometry_msgs::msg::Twist();
    smooth_twist_ = geometry_msgs::msg::Twist();
    last_filter_update_ = std::chrono::steady_clock::now();
    publisher_->publish(smooth_twist_);
  }

  void update_target_twist(double filter_dt_s)
  {
    geometry_msgs::msg::Twist cmd;
    const bool enabled = enable_button_ < 0 || button_pressed(enable_button_);
    const bool stop = button_pressed(stop_button_);

    if (enabled && !stop) {

      double linear_x = chassis::remote_apply_deadzone(axis_value(axis_linear_), deadzone_);
      double linear_y = chassis::remote_apply_deadzone(axis_value(axis_lateral_), deadzone_);
      double angular_z = chassis::remote_apply_deadzone(axis_value(axis_angular_), deadzone_);
      if (invert_linear_axis_) {linear_x = -linear_x;}
      if (invert_lateral_axis_) {linear_y = -linear_y;}
      if (invert_angular_axis_) {angular_z = -angular_z;}

      const std::array<double, 3> input_axes{linear_x, linear_y, angular_z};
      std::array<double, 3> motion_axes{};
      for (std::size_t i = 0; i < filtered_axes_.size(); ++i) {
        filtered_axes_[i] = chassis::remote_low_pass_axis(
          filtered_axes_[i], input_axes[i], filter_dt_s, joystick_low_pass_time_constant_s_);
        // The existing normalized-axis deadzone makes the decay reach an exact zero command.
        // Keep the filter state itself so small steady inputs can accumulate.
        motion_axes[i] = chassis::remote_apply_deadzone(filtered_axes_[i], deadzone_);
      }

      const auto motion = chassis::remote_motion_from_axes(
        motion_axes[0], motion_axes[1], motion_axes[2], remote_config_);
      cmd.linear.x = motion.linear_x_m_s;
      cmd.linear.y = motion.linear_y_m_s;
      cmd.angular.z = motion.angular_z_rad_s;
    } else {
      // Stop/enable buttons must not wait for the input filter to decay.
      filtered_axes_.fill(0.0);
    }

    target_twist_ = cmd;
  }

  void timer_callback()
  {
    arm_action_.reset();
    // Read only a snapshot here: a stalled Bluetooth request must never block zero output.
    const auto link = link_monitor_->snapshot();
    if (!link.healthy) {
      if (!link_warning_reported_) {
        RCLCPP_WARN(get_logger(), "Joystick link unavailable: %s; zero velocity; recenter to resume",
          link.reason.c_str());
        link_warning_reported_ = true;
      }
      if (joy_fd_ >= 0 || link_was_healthy_) {invalidate_joystick("joystick link lost");}
      link_was_healthy_ = false;
      publish_zero_velocity();
      return;
    }
    if (!link_was_healthy_ || link.generation != link_generation_) {
      // Reopen to obtain fresh initialization states; cached neutral values cannot rearm.
      invalidate_joystick("joystick link reconnected");
      next_reconnect_ = {};
      link_generation_ = link.generation;
      link_was_healthy_ = true;
      link_warning_reported_ = false;
      RCLCPP_INFO(get_logger(), "Joystick link verified; waiting for neutral initialization");
    }
    read_joystick_events();
    if (emergency_stopped_) {return;}

    if (calibration_finish_pending_ && session_) {
      if (session_->stage == RemoteControlSession::Stage::waiting) {
        calibration_finish_pending_ = false;
      } else if (session_->stage == RemoteControlSession::Stage::running) {
        // Main enters running only after hardware startup and calibration succeed.
        std_msgs::msg::String message;
        message.data = "结束校准";
        arm_speech_publisher_->publish(message);
        calibration_finish_pending_ = false;
      }
    }

    if (startup_pending()) {
      update_arm_current(false);
      joystick_armed_ = false;
      arm_joystick_.disarm();
      arm_action_.reset();
      publish_zero_velocity();
      return;
    }

    const auto filter_now = std::chrono::steady_clock::now();
    const bool stale = joy_fd_ < 0 || !joy_seen_ ||
      (timeout_sec_ > 0.0 &&
      std::chrono::duration<double>(filter_now - last_joy_time_).count() > timeout_sec_);
    if (stale) {
      if (joystick_armed_) {
        RCLCPP_WARN(get_logger(), "Joystick input timed out: zero velocity; recenter to resume");
      }
      joystick_armed_ = false;
      update_arm_current(false);
      arm_joystick_.disarm();
      cancel_arm_adjustment("joystick unavailable");
      publish_zero_velocity();
      return;
    }
    const double filter_dt_s =
      std::chrono::duration<double>(filter_now - last_filter_update_).count();
    last_filter_update_ = filter_now;
    // Check after draining initialization events: unseen axes are not neutral.
    // A held stick or an accidentally selected trigger must not arm at startup.
    if (!joystick_armed_ && joy_fd_ >= 0) {
      const auto centered = [this](int index) {
          return index == -1 || (index >= 0 && static_cast<size_t>(index) < axes_.size() &&
                 axis_seen_[index] &&
                 chassis::remote_apply_deadzone(axes_[index], deadzone_) == 0.0);
        };
      if (centered(axis_linear_) && centered(axis_lateral_) && centered(axis_angular_) &&
        (!enable_arm_controls_ || arm_joystick_.neutral_ready()))
      {
        joystick_armed_ = true;
        RCLCPP_INFO(get_logger(), "Joystick centered: motion input enabled");
      }
    }
    if (!joystick_armed_) {
      update_arm_current(false);
      arm_joystick_.disarm();
      cancel_arm_adjustment("joystick not centered");
      publish_zero_velocity();
      return;
    }
    update_arm_current(
      !button_pressed(stop_button_) &&
      (enable_button_ < 0 || button_pressed(enable_button_)));
    if (button_pressed(stop_button_) ||
      (enable_button_ >= 0 && !button_pressed(enable_button_)))
    {
      arm_joystick_.disarm();
      cancel_arm_adjustment("joystick inactive");
    } else if (enable_arm_controls_) {
      if (!arm_joystick_.armed() && !arm_joystick_.arm()) {
        cancel_arm_adjustment("Arm controls not neutral");
      } else {
        // Finish the previous response first so a ready 100 ms repeat can start
        // this tick. Only one request is in flight; busy repeats are discarded.
        update_arm_adjustment();
        const auto repeated = arm_joystick_.repeat();
        if (repeated.direction != 0) {
          handle_arm_action(repeated);
          update_arm_adjustment();
        }
      }
    }
    // Update once per timer tick, including ticks without joystick events.
    update_target_twist(filter_dt_s);
    const auto cmd = target_twist_;

    const double dt = 0.05;

    smooth_twist_.linear.x = limit_rate(
      smooth_twist_.linear.x,
      cmd.linear.x,
      max_accel_ * dt,
      max_decel_ * dt
    );

    smooth_twist_.linear.y = limit_rate(
      smooth_twist_.linear.y,
      cmd.linear.y,
      max_accel_ * dt,
      max_decel_ * dt
    );

    smooth_twist_.angular.z = cmd.angular.z;
    publisher_->publish(smooth_twist_);
  }

  std::shared_ptr<RemoteControlSession> session_;
  const bool enable_arm_controls_;
  int start_button_{14};
  bool start_released_{false};
  bool emergency_stop_released_{false};
  bool calibration_finish_pending_{false};
  std::chrono::steady_clock::time_point next_reconnect_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string device_path_;
  std::shared_ptr<JoystickLinkMonitor> link_monitor_;
  std::uint64_t link_generation_{0};
  bool link_was_healthy_{false};
  bool link_warning_reported_{false};
  int joy_fd_{-1};
  std::vector<double> axes_;
  std::vector<bool> axis_seen_;
  bool joystick_armed_{false};
  std::vector<int> buttons_;

  geometry_msgs::msg::Twist target_twist_;
  geometry_msgs::msg::Twist smooth_twist_;
  std::chrono::steady_clock::time_point last_joy_time_{};
  bool joy_seen_{false};
  double max_accel_{0.6};
  double max_decel_{1.2};
  int axis_linear_{3};
  int axis_lateral_{0};
  int axis_angular_{6};
  int enable_button_{-1};
  int stop_button_{0};
  int emergency_stop_button_{11};
  bool emergency_stopped_{false};
  bool invert_linear_axis_{true};
  bool invert_lateral_axis_{true};
  bool invert_angular_axis_{true};
  chassis::RemoteKinematicsConfig remote_config_{};
  std::array<double, 3> filtered_axes_{};
  std::chrono::steady_clock::time_point last_filter_update_{std::chrono::steady_clock::now()};
  double joystick_low_pass_time_constant_s_{0.1};
  double deadzone_{0.05};
  double timeout_sec_{0.0};
  chassis::ArmJoystick arm_joystick_;
  std::optional<chassis::ArmJoystickAction> arm_action_;
  rclcpp::Client<GetParameters>::SharedPtr arm_get_client_;
  rclcpp::Client<SetParameters>::SharedPtr arm_set_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arm_pose_subscription_;
  std::optional<std::array<double, 3>> arm_actual_pose_;
  std::chrono::steady_clock::time_point arm_pose_received_{};
  std::optional<rclcpp::Client<GetParameters>::FutureAndRequestId> arm_get_request_;
  std::optional<rclcpp::Client<SetParameters>::FutureAndRequestId> arm_set_request_;
  std::chrono::steady_clock::time_point arm_deadline_{};
  unsigned arm_requested_coordinate_{0};
  bool arm_requested_repeat_{false};
  double arm_requested_delta_{0};
  std::array<double, 3> arm_requested_pose_{};
  double arm_translation_step_m_{0.01};
  double arm_pitch_step_rad_{0.017453292519943295};
  bool invert_arm_direction_{false};
  rclcpp::Publisher<rcl_interfaces::msg::Parameter>::SharedPtr arm_current_publisher_;
  rclcpp::Context::SharedPtr arm_speech_context_;
  rclcpp::Node::SharedPtr arm_speech_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_speech_publisher_;
  int arm_current_permille_{50};
  int last_arm_current_{0};
  bool arm_current_ready_{false};
  std::array<bool, 2> arm_current_seen_{};
};

}  // namespace

std::shared_ptr<rclcpp::Node> make_remote_control(
  const rclcpp::NodeOptions & options, std::shared_ptr<RemoteControlSession> session,
  bool enable_arm_controls, const std::string & default_steering_config_file,
  std::shared_ptr<JoystickLinkMonitor> link_monitor)
{
  return std::make_shared<RemoteCtrl>(
    options, std::move(session), enable_arm_controls, default_steering_config_file,
    std::move(link_monitor));
}

}  // namespace chassis
