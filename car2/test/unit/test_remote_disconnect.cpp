#include "example/remote_control.h"
#include "example/joystick_link_monitor.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fcntl.h>
#include <iostream>
#include <linux/joystick.h>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"

namespace
{
using namespace std::chrono_literals;
using Twist = geometry_msgs::msg::Twist;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

bool zero(const Twist & command)
{
  return command.linear.x == 0.0 && command.linear.y == 0.0 && command.linear.z == 0.0 &&
         command.angular.x == 0.0 && command.angular.y == 0.0 && command.angular.z == 0.0;
}

bool moving(const Twist & command)
{
  return command.linear.x > 0.0 && command.linear.y > 0.0 && command.angular.z > 0.0;
}

class RemoteFixture
{
public:
  RemoteFixture(
    bool connected, bool waiting, double timeout_s = 0.0, bool arm_controls = false,
    std::vector<rclcpp::Parameter> extra_parameters = {})
  {
    char pattern[] = "/tmp/chassis-remote-test-XXXXXX";
    const char * directory = ::mkdtemp(pattern);
    require(directory != nullptr, "cannot create joystick test directory");
    directory_ = directory;
    device_ = directory_ / "joystick";
    if (connected) {connect();}

    auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
    std::vector<rclcpp::Parameter> parameters{
      rclcpp::Parameter("device_path", device_.string()),
      rclcpp::Parameter("cmd_vel_topic", "/remote_disconnect_test/cmd_vel"),
      rclcpp::Parameter("arm_position_topic", "/remote_disconnect_test/arm_jog"),
      rclcpp::Parameter("arm_pose_topic", "/remote_disconnect_test/arm_pose"),
      rclcpp::Parameter("arm_node", "/remote_disconnect_test_arm"),
      rclcpp::Parameter("invert_linear_axis", false),
      rclcpp::Parameter("invert_lateral_axis", false),
      rclcpp::Parameter("invert_angular_axis", false),
      rclcpp::Parameter("max_accel", 0.2),
      rclcpp::Parameter("max_decel", 0.01)};
    if (timeout_s > 0.0) {parameters.emplace_back("timeout_sec", timeout_s);}
    parameters.insert(parameters.end(), extra_parameters.begin(), extra_parameters.end());
    options.parameter_overrides(parameters);
    if (waiting) {session_ = std::make_shared<chassis::RemoteControlSession>();}
    link_available_ = std::make_shared<std::atomic<bool>>(true);
    link_monitor_ = std::make_shared<chassis::JoystickLinkMonitor>(
      [available = link_available_] {return available->load();});
    remote_ = chassis::make_remote_control(options, session_, arm_controls, "", link_monitor_);
    observer_ = std::make_shared<rclcpp::Node>(
      "remote_disconnect_test", rclcpp::NodeOptions().use_intra_process_comms(true));
    subscription_ = observer_->create_subscription<Twist>(
      "/remote_disconnect_test/cmd_vel", 10,
      [this](Twist::ConstSharedPtr command) {commands.push_back(*command);});
    if (arm_controls) {
      tracking_subscription_ = observer_->create_subscription<std_msgs::msg::Bool>(
        "/arm_tracking_requested", 10,
        [this](std_msgs::msg::Bool::ConstSharedPtr message) {
          tracking_requests.push_back(message->data);
        });
      get_service_ = observer_->create_service<rcl_interfaces::srv::GetParameters>(
        "/remote_disconnect_test_arm/get_parameters",
        [this](const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request>,
        std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> response) {
          ++manual_gets;
          response->values.push_back(
            rclcpp::Parameter("yiyou_arm_target", std::vector<double>{}).to_parameter_msg().value);
        });
      set_service_ = observer_->create_service<rcl_interfaces::srv::SetParametersAtomically>(
        "/remote_disconnect_test_arm/set_parameters_atomically",
        [this](const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request>,
        std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> response) {
          ++manual_sets;
          response->result.successful = true;
        });
      arm_subscription_ = observer_->create_subscription<std_msgs::msg::Int8>(
        "/remote_disconnect_test/arm_jog", 10,
        [this](std_msgs::msg::Int8::ConstSharedPtr command) {jog_commands.push_back(command->data);});
      pose_publisher_ = observer_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/remote_disconnect_test/arm_pose", 1);
      pose_timer_ = observer_->create_wall_timer(50ms, [this] {
          if (!publish_pose_) {return;}
          geometry_msgs::msg::PoseStamped message;
          message.header.frame_id = "arm_shoulder";
          message.pose.orientation.w = 1.0;
          pose_publisher_->publish(message);
        });
    }
    executor_.add_node(remote_);
    executor_.add_node(observer_);
  }

  ~RemoteFixture()
  {
    disconnect();
    executor_.remove_node(remote_);
    executor_.remove_node(observer_);
    remote_.reset();
    subscription_.reset();
    observer_.reset();
    std::error_code error;
    std::filesystem::remove_all(directory_, error);
  }

  void connect()
  {
    if (!std::filesystem::exists(device_)) {
      require(::mkfifo(device_.c_str(), 0600) == 0, "cannot create joystick FIFO");
    }
    require(writer_ < 0, "joystick writer is already connected");
    // Holding the write end prevents EOF; closing it simulates device removal.
    writer_ = ::open(device_.c_str(), O_RDWR | O_NONBLOCK);
    require(writer_ >= 0, "cannot open joystick FIFO writer");
  }

  void disconnect()
  {
    if (writer_ >= 0) {
      ::close(writer_);
      writer_ = -1;
    }
  }

  void link_available(bool available) {link_available_->store(available);}

  void axes(std::int16_t value, bool initialization = false)
  {
    require(writer_ >= 0, "cannot send events to a disconnected joystick");
    for (const unsigned char axis : {0, 3, 6}) {
      js_event event{};
      event.type = JS_EVENT_AXIS | (initialization ? JS_EVENT_INIT : 0);
      event.number = axis;
      event.value = value;
      require(
        ::write(writer_, &event, sizeof(event)) == static_cast<ssize_t>(sizeof(event)),
        "cannot write joystick event");
    }
  }

  void clear()
  {
    // Drain a command already queued by the timer before changing connection state.
    executor_.spin_some();
    executor_.spin_some();
    commands.clear();
    jog_commands.clear();
    tracking_requests.clear();
  }

  void button(unsigned char number, bool pressed, bool initialization = false)
  {
    js_event event{};
    event.type = JS_EVENT_BUTTON | (initialization ? JS_EVENT_INIT : 0);
    event.number = number;
    event.value = pressed ? 1 : 0;
    require(::write(writer_, &event, sizeof(event)) == static_cast<ssize_t>(sizeof(event)),
      "cannot write joystick button event");
  }

  void arm_axis(std::int16_t value, bool initialization = false)
  {
    js_event event{};
    event.type = JS_EVENT_AXIS | (initialization ? JS_EVENT_INIT : 0);
    event.number = 7;
    event.value = value;
    require(::write(writer_, &event, sizeof(event)) == static_cast<ssize_t>(sizeof(event)),
      "cannot write arm axis event");
  }

  void initialize_arm(bool positive_held = false, bool tracking_held = false)
  {
    axes(0, true);
    arm_axis(0, true);
    for (const unsigned char number : {0, 1, 2, 3, 4, 6}) {
      button(number, (number == 0 && positive_held) || (number == 6 && tracking_held), true);
    }
  }

  void stage(chassis::RemoteControlSession::Stage value)
  {
    require(session_ != nullptr, "fixture has no startup session");
    session_->stage = value;
  }

  bool tracking_publisher_exists() const
  {
    for (const auto & endpoint : remote_->get_publishers_info_by_topic("/arm_tracking_requested")) {
      if (endpoint.node_name() == remote_->get_name()) {return true;}
    }
    return false;
  }

  void require_tracking(bool expected, const char * message, std::chrono::milliseconds duration = 150ms)
  {
    clear();
    observe_for(duration);
    require(tracking_requests.size() >= 2, "tracking request must refresh its heartbeat");
    require(std::all_of(tracking_requests.begin(), tracking_requests.end(),
      [expected](bool requested) {return requested == expected;}), message);
  }

  void wait_for_tracking(bool expected, const char * message,
    std::chrono::milliseconds duration = 300ms)
  {
    clear();
    wait_for([this, expected] {
        return !tracking_requests.empty() && tracking_requests.back() == expected;
      }, message, duration);
  }

  void publish_pose(bool enabled) {publish_pose_ = enabled;}

  void require_jog_zeros(const char * message, std::chrono::milliseconds duration = 200ms)
  {
    clear();
    observe_for(duration);
    require(jog_commands.size() >= 2, "arm neutral must refresh zero commands");
    require(std::all_of(jog_commands.begin(), jog_commands.end(), [](int jog) {return jog == 0;}),
      message);
  }

  void wait_for_jog(int expected, const char * message, std::chrono::milliseconds limit = 300ms)
  {
    wait_for([this, expected] {return !jog_commands.empty() && jog_commands.back() == expected;},
      message, limit);
  }

  template<typename Predicate>
  void wait_for(Predicate predicate, const char * message, std::chrono::milliseconds limit = 1500ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (predicate()) {return;}
      std::this_thread::sleep_for(2ms);
    }
    throw std::runtime_error(message);
  }

  void observe_for(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(2ms);
    }
    executor_.spin_some();
  }

  void require_zeros(const char * message, std::chrono::milliseconds duration = 250ms)
  {
    clear();
    observe_for(duration);
    require(commands.size() >= 3, "remote must continuously publish commands");
    require(std::all_of(commands.begin(), commands.end(), zero), message);
  }

  void arm_and_move()
  {
    axes(0, true);
    require_zeros("neutral initialization must publish zero");
    axes(32767);
    clear();
    wait_for(
      [this] {
        return !commands.empty() && moving(commands.back()) &&
        commands.back().linear.x >= 0.04;
      },
      "joystick must produce movement after neutral initialization");
  }

  std::vector<Twist> commands;
  std::vector<int> jog_commands;
  std::vector<bool> tracking_requests;
  unsigned manual_gets{0};
  unsigned manual_sets{0};

private:
  std::filesystem::path directory_;
  std::filesystem::path device_;
  int writer_{-1};
  std::shared_ptr<chassis::RemoteControlSession> session_;
  std::shared_ptr<std::atomic<bool>> link_available_;
  std::shared_ptr<chassis::JoystickLinkMonitor> link_monitor_;
  std::shared_ptr<rclcpp::Node> remote_;
  std::shared_ptr<rclcpp::Node> observer_;
  rclcpp::Subscription<Twist>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr arm_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tracking_subscription_;
  rclcpp::Service<rcl_interfaces::srv::GetParameters>::SharedPtr get_service_;
  rclcpp::Service<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr set_service_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
  bool publish_pose_{true};
  rclcpp::executors::SingleThreadedExecutor executor_;
};

void missing_joystick_publishes_zero(bool waiting)
{
  RemoteFixture fixture(false, waiting);
  fixture.require_zeros("missing joystick must publish zero while waiting or running");
}

void unplug_stops_immediately_and_reconnect_requires_neutral()
{
  RemoteFixture fixture(true, false);
  fixture.arm_and_move();
  fixture.clear();
  fixture.disconnect();
  fixture.wait_for(
    [&fixture] {return !fixture.commands.empty();}, "no command after disconnect", 250ms);
  require(zero(fixture.commands.front()), "first command after disconnect must be exactly zero");
  fixture.require_zeros("disconnected joystick must keep publishing zero");

  fixture.connect();
  fixture.axes(32767, true);
  fixture.require_zeros("reconnecting with held axes must not move", 1200ms);
  fixture.axes(0);
  fixture.require_zeros("reconnected joystick must remain stopped while neutral");
  fixture.axes(32767);
  fixture.wait_for(
    [&fixture] {return !fixture.commands.empty() && moving(fixture.commands.back());},
    "joystick must recover after reporting neutral");

  fixture.clear();
  fixture.observe_for(800ms);
  require(fixture.commands.size() >= 10, "held joystick must keep publishing commands");
  require(
    std::all_of(fixture.commands.begin(), fixture.commands.end(), moving),
    "default timeout must not stop a held joystick merely because no new events arrive");
}

void configured_timeout_stops_immediately_and_requires_neutral()
{
  RemoteFixture fixture(true, false, 0.4);
  fixture.arm_and_move();
  fixture.clear();
  fixture.wait_for(
    [&fixture] {
      return std::any_of(
        fixture.commands.begin(), fixture.commands.end(),
        [](const Twist & command) {return command.angular.z == 0.0;});
    },
    "configured joystick timeout must stop motion", 1000ms);
  const auto stopped = std::find_if(
    fixture.commands.begin(), fixture.commands.end(),
    [](const Twist & command) {return command.angular.z == 0.0;});
  require(zero(*stopped), "first command at timeout must bypass the speed ramp and be all zero");

  fixture.axes(32767);
  fixture.require_zeros("fresh held-axis events after timeout must wait for neutral");
  fixture.axes(0);
  fixture.require_zeros("returning to neutral after timeout must remain stopped");
  fixture.axes(32767);
  fixture.wait_for(
    [&fixture] {return !fixture.commands.empty() && moving(fixture.commands.back());},
    "joystick must recover from timeout after returning to neutral");
}

void radio_loss_with_live_device_stops_and_requires_fresh_neutral()
{
  RemoteFixture fixture(true, false);
  fixture.arm_and_move();
  fixture.clear();
  // Keep the joystick FIFO open: wireless failure precedes Linux device removal.
  fixture.link_available(false);
  fixture.wait_for(
    [&fixture] {return !fixture.commands.empty() && zero(fixture.commands.back());},
    "radio loss must stop even while the joystick device is present", 450ms);
  fixture.require_zeros("failed link must continuously publish exact zero");

  fixture.link_available(true);
  fixture.axes(32767, true);
  fixture.require_zeros("link recovery with held axes must not restore old motion", 500ms);
  fixture.axes(0);
  fixture.require_zeros("neutral after link recovery must remain stopped");
  fixture.axes(32767);
  fixture.wait_for(
    [&fixture] {return !fixture.commands.empty() && moving(fixture.commands.back());},
    "verified link must allow motion only after fresh neutral");
}

void unverified_link_never_allows_motion()
{
  RemoteFixture fixture(true, false);
  fixture.link_available(false);
  fixture.observe_for(200ms);
  fixture.axes(0, true);
  fixture.require_zeros("unverified link must not arm from neutral");
  fixture.axes(32767);
  fixture.require_zeros("unverified link must not send nonzero commands");
}

void recovered_link_cannot_hide_failure_between_ros_ticks()
{
  RemoteFixture fixture(true, false);
  fixture.arm_and_move();
  fixture.clear();
  // Do not spin ROS: the background monitor observes a failure and recovery in between ticks.
  fixture.link_available(false);
  std::this_thread::sleep_for(180ms);
  fixture.link_available(true);
  std::this_thread::sleep_for(180ms);
  fixture.axes(32767, true);
  fixture.require_zeros("recovered generation must invalidate held input after a delayed tick");
}
void position_jog_requires_release_and_stops_on_conflicting_buttons()
{
  RemoteFixture fixture(true, false, 0.0, true);
  fixture.initialize_arm(true);
  fixture.require_jog_zeros("held fourth-axis button at initialization must not move", 300ms);
  fixture.button(0, false);
  fixture.require_jog_zeros("released fourth-axis buttons must stay stopped");
  fixture.button(0, true);
  fixture.wait_for_jog(1, "button0 must request positive position jog");
  fixture.clear();
  fixture.observe_for(150ms);
  require(fixture.jog_commands.size() >= 2, "held position jog must refresh its lease");
  require(std::all_of(fixture.jog_commands.begin(), fixture.jog_commands.end(),
    [](int direction) {return direction == 1;}), "held button0 must maintain positive jog");

  fixture.clear();
  fixture.button(0, false);
  fixture.wait_for_jog(0, "releasing button0 must publish halt");
  require(fixture.jog_commands.front() == 0, "first fourth-axis command after release must halt");
  fixture.button(4, true);
  fixture.wait_for_jog(-1, "button4 must request negative position jog");
  fixture.button(0, true);
  fixture.wait_for_jog(0, "simultaneous fourth-axis buttons must halt");
  fixture.button(4, false);
  fixture.require_jog_zeros("one held button after a conflict must not rearm position jog");
  fixture.button(0, false);
  fixture.require_jog_zeros("both buttons must be released before rearming");
  fixture.button(0, true);
  fixture.wait_for_jog(1, "position jog must resume after both buttons were released");
}

void position_jog_stale_feedback_and_disconnect_require_release()
{
  RemoteFixture fixture(true, false, 0.0, true);
  fixture.initialize_arm();
  fixture.require_jog_zeros("position jog starts neutral", 300ms);
  fixture.button(0, true);
  fixture.wait_for_jog(1, "positive position jog must start");
  fixture.publish_pose(false);
  fixture.clear();
  fixture.wait_for_jog(0, "stale arm feedback must halt position jog", 800ms);
  fixture.publish_pose(true);
  fixture.require_jog_zeros("fresh feedback must not restart a still-held button");
  fixture.button(0, false);
  fixture.require_jog_zeros("release must rearm after stale feedback");
  fixture.button(4, true);
  fixture.wait_for_jog(-1, "negative position jog must start after release");

  fixture.clear();
  fixture.disconnect();
  fixture.wait_for_jog(0, "joystick disconnect must halt position jog");
  require(fixture.jog_commands.front() == 0,
    "first fourth-axis command after disconnect must halt");
  fixture.connect();
  fixture.initialize_arm(true);
  fixture.require_jog_zeros("reconnected held position button must not move", 1200ms);
  fixture.button(0, false);
  fixture.require_jog_zeros("reconnected position controls require release");
  fixture.button(0, true);
  fixture.wait_for_jog(1, "position jog must recover after reconnection and release");

  fixture.clear();
  fixture.link_available(false);
  fixture.wait_for_jog(0, "radio loss with live device must halt position jog", 450ms);
}

void tracking_button_requires_release_and_only_toggles_on_edges()
{
  RemoteFixture fixture(true, false, 0.0, true);
  fixture.initialize_arm(false, true);
  fixture.require_tracking(false, "held tracking button at initialization must not start", 250ms);
  fixture.button(6, true);
  fixture.require_tracking(false, "repeated held initialization button must not start");
  fixture.button(6, false);
  fixture.button(6, true);
  fixture.wait_for_tracking(true, "released tracking button must enter on the next rising edge");
  fixture.button(6, true);
  fixture.button(6, true);
  fixture.require_tracking(true, "held tracking button must not repeatedly toggle");
  fixture.button(6, false);
  fixture.require_tracking(true, "release alone must not cancel tracking");
  fixture.button(6, true);
  fixture.wait_for_tracking(false, "second tracking press must cancel");
  fixture.require_tracking(false, "cancelled tracking must remain off");

  fixture.button(2, true);
  fixture.button(2, false);
  fixture.require_tracking(false, "button2 must no longer request tracking");

  fixture.button(6, false);
  // Queue both presses before spinning: one timer read drains this entire batch.
  for (int press = 0; press < 2; ++press) {
    fixture.button(6, true);
    fixture.button(6, false);
  }
  fixture.require_tracking(false, "two tracking presses in one batch must remain off");
  fixture.button(6, true);
  fixture.button(6, false, true);
  fixture.require_tracking(false, "initialization must clear an earlier queued tracking edge");

  fixture.button(6, true);
  fixture.wait_for_tracking(true, "fresh press after initialization must enter tracking");
  fixture.button(6, false);
  for (int press = 0; press < 2; ++press) {
    fixture.button(6, true);
    fixture.button(6, false);
  }
  fixture.require_tracking(true, "two tracking presses in one batch must preserve the on state");
}

void tracking_waits_for_startup_and_neutral_without_queuing_presses()
{
  {
    RemoteFixture fixture(true, true, 0.0, true);
    fixture.initialize_arm();
    fixture.require_tracking(false, "waiting for startup must keep tracking off", 200ms);
    fixture.button(6, true);
    fixture.require_tracking(false, "tracking press while waiting must be discarded");
    fixture.stage(chassis::RemoteControlSession::Stage::starting);
    fixture.button(6, false);
    fixture.button(6, true);
    fixture.require_tracking(false, "tracking press during calibration must be discarded");
    fixture.stage(chassis::RemoteControlSession::Stage::running);
    fixture.require_tracking(false, "finishing calibration must not replay a tracking press");
    fixture.button(6, false);
    fixture.button(6, true);
    fixture.wait_for_tracking(true, "fresh press after calibration must enter tracking");
  }
  {
    RemoteFixture fixture(true, false, 0.0, true);
    fixture.initialize_arm();
    fixture.axes(32767, true);
    fixture.button(6, true);
    fixture.require_tracking(false, "uncentered joystick must reject tracking", 250ms);
    fixture.axes(0);
    fixture.require_tracking(false, "centering later must not replay rejected tracking press");
    fixture.button(6, false);
    fixture.button(6, true);
    fixture.wait_for_tracking(true, "fresh press after centering must enter tracking");
  }
}

void tracking_stop_and_enable_buttons_cancel_without_resuming()
{
  RemoteFixture fixture(true, false, 0.0, true, {
      rclcpp::Parameter("stop_button", 5), rclcpp::Parameter("enable_button", 7)});
  fixture.initialize_arm();
  fixture.button(7, true);
  fixture.require_tracking(false, "tracking starts off when joystick is enabled", 200ms);
  fixture.button(6, true);
  fixture.wait_for_tracking(true, "tracking must enter before stop test");
  fixture.button(5, true);
  fixture.wait_for_tracking(false, "stop button must cancel tracking");
  fixture.button(5, false);
  fixture.button(6, true);
  fixture.require_tracking(false, "stop release must not resume held tracking request");
  fixture.button(6, false);
  fixture.button(6, true);
  fixture.wait_for_tracking(true, "fresh press after stop release must enter tracking");
  fixture.button(7, false);
  fixture.wait_for_tracking(false, "releasing enable must cancel tracking");
  fixture.button(7, true);
  fixture.button(6, true);
  fixture.require_tracking(false, "enable recovery must not resume held tracking request");
}

void tracking_disconnect_and_timeout_require_a_fresh_press()
{
  {
    RemoteFixture fixture(true, false, 0.0, true);
    fixture.initialize_arm();
    fixture.require_tracking(false, "tracking starts off", 200ms);
    fixture.button(6, true);
    fixture.wait_for_tracking(true, "tracking must enter before disconnect test");
    fixture.disconnect();
    fixture.wait_for_tracking(false, "device disconnection must cancel tracking");
    fixture.connect();
    fixture.initialize_arm(false, true);
    fixture.require_tracking(false, "reconnected held tracking button must not resume", 1200ms);
    fixture.button(6, false);
    fixture.button(6, true);
    fixture.wait_for_tracking(true, "fresh press after reconnect must enter tracking");
    fixture.link_available(false);
    fixture.wait_for_tracking(false, "radio loss with live device must cancel tracking", 450ms);
    fixture.link_available(true);
    fixture.initialize_arm(false, true);
    fixture.require_tracking(false, "radio recovery must not restore tracking", 500ms);
    fixture.button(6, false);
    fixture.button(6, true);
    fixture.wait_for_tracking(true, "fresh press after radio recovery must enter tracking");
  }
  {
    RemoteFixture fixture(true, false, 0.35, true);
    fixture.initialize_arm();
    fixture.require_tracking(false, "tracking starts off before timeout test", 150ms);
    fixture.button(6, true);
    fixture.wait_for_tracking(true, "tracking must enter before timeout test");
    fixture.wait_for_tracking(false, "event timeout must cancel tracking", 700ms);
    fixture.axes(0);
    fixture.require_tracking(false, "fresh neutral alone must not resume tracking");
    fixture.button(6, false);
    fixture.button(6, true);
    fixture.wait_for_tracking(true, "fresh press after timeout must enter tracking");
  }
}

void tracking_reserves_chassis_but_preserves_fourth_axis()
{
  RemoteFixture fixture(true, false, 0.0, true);
  fixture.initialize_arm();
  fixture.require_tracking(false, "tracking starts off before coexistence test", 200ms);
  fixture.button(6, true);
  fixture.wait_for_tracking(true, "tracking must enter before manual suppression test");
  fixture.require_zeros("visual tracking must publish exact neutral chassis heartbeats");
  fixture.arm_axis(-32767);
  fixture.observe_for(200ms);
  require(fixture.manual_gets == 0 && fixture.manual_sets == 0,
    "tracking must suppress planar manual parameter requests");
  fixture.button(0, true);
  fixture.wait_for_jog(1, "tracking must preserve fourth-axis jog");
  fixture.axes(32767);
  fixture.wait_for_tracking(false, "chassis stick intervention must cancel visual tracking");
  fixture.wait_for([&fixture] {return !fixture.commands.empty() && moving(fixture.commands.back());},
    "manual intervention must reach the controller's ownership gate");
  fixture.button(6, false);
  fixture.button(6, true);
  fixture.require_tracking(false, "held chassis sticks must prevent tracking re-entry");
  fixture.observe_for(150ms);
  require(fixture.manual_gets == 0 && fixture.manual_sets == 0,
    "held planar stick must not replay after tracking exit");
  fixture.arm_axis(0);
  fixture.observe_for(100ms);
  fixture.arm_axis(-32767);
  fixture.wait_for([&fixture] {return fixture.manual_sets > 0;},
    "planar manual control must recover after centering and a fresh step");
}

void tracking_entry_clears_the_previous_chassis_ramp()
{
  RemoteFixture fixture(true, false, 0.0, true);
  fixture.initialize_arm();
  fixture.require_tracking(false, "tracking starts off before ramp test", 200ms);
  fixture.axes(32767);
  fixture.wait_for([&fixture] {
      return !fixture.commands.empty() && moving(fixture.commands.back()) &&
             fixture.commands.back().linear.x >= 0.04;
    }, "manual ramp must be moving before tracking entry");
  fixture.button(6, true);
  fixture.require_tracking(false, "tracking must not start with a moving chassis stick");
  fixture.axes(0);
  fixture.require_tracking(false, "neutral alone must not replay a rejected tracking press");
  fixture.button(6, false);
  fixture.button(6, true);
  fixture.wait_for_tracking(true, "fresh press with centered chassis sticks must enable tracking");
  fixture.require_zeros("tracking entry must clear the slow manual deceleration ramp");
}

void tracking_button_configuration_is_validated()
{
  const auto rejected = [](std::vector<rclcpp::Parameter> parameters) {
      bool invalid = false;
      try {
        chassis::make_remote_control(
          rclcpp::NodeOptions().parameter_overrides(parameters), nullptr, true, "");
      } catch (const std::invalid_argument & error) {
        invalid = std::string(error.what()).find("arm_tracking_button") != std::string::npos;
      }
      require(invalid, "invalid or conflicting tracking button must be rejected");
    };
  for (const int button : {-1, 256, 0, 1, 3, 4, 11, 14}) {
    rejected({rclcpp::Parameter("arm_tracking_button", button)});
  }
  for (const auto * name : {"enable_button", "stop_button", "emergency_stop_button", "start_button"}) {
    rejected({rclcpp::Parameter(name, 6)});
  }

  char pattern[] = "/tmp/chassis-tracking-button-test-XXXXXX";
  const auto directory = ::mkdtemp(pattern);
  require(directory != nullptr, "cannot create tracking button test directory");
  const auto device = std::filesystem::path(directory) / "joystick";
  require(::mkfifo(device.c_str(), 0600) == 0, "cannot create tracking button joystick FIFO");
  try {
    // FIFO ioctl falls back to 16 buttons; index16 must fail before any device events.
    rejected({rclcpp::Parameter("device_path", device.string()),
      rclcpp::Parameter("arm_tracking_button", 16)});
  } catch (...) {
    std::filesystem::remove_all(directory);
    throw;
  }
  std::filesystem::remove_all(directory);
}

void chassis_only_has_no_tracking_publisher()
{
  RemoteFixture fixture(true, false, 0.0, false, {
      rclcpp::Parameter("arm_tracking_button", 0)});
  fixture.observe_for(150ms);
  require(!fixture.tracking_publisher_exists(),
    "chassis-only remote must not create a tracking request publisher");
}

void stale_current_override_is_rejected()
{
  auto options = rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("arm_current_permille", 500)});
  bool rejected = false;
  try {
    chassis::make_remote_control(options, nullptr, true, "");
  } catch (const std::invalid_argument & error) {
    rejected = std::string(error.what()).find("arm_current_permille is retired") != std::string::npos;
  }
  require(rejected, "old current amplitude must not silently configure position jog");
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    missing_joystick_publishes_zero(true);
    missing_joystick_publishes_zero(false);
    unplug_stops_immediately_and_reconnect_requires_neutral();
    configured_timeout_stops_immediately_and_requires_neutral();
    radio_loss_with_live_device_stops_and_requires_fresh_neutral();
    unverified_link_never_allows_motion();
    recovered_link_cannot_hide_failure_between_ros_ticks();
    position_jog_requires_release_and_stops_on_conflicting_buttons();
    position_jog_stale_feedback_and_disconnect_require_release();
    tracking_button_requires_release_and_only_toggles_on_edges();
    tracking_waits_for_startup_and_neutral_without_queuing_presses();
    tracking_stop_and_enable_buttons_cancel_without_resuming();
    tracking_disconnect_and_timeout_require_a_fresh_press();
    tracking_reserves_chassis_but_preserves_fourth_axis();
    tracking_entry_clears_the_previous_chassis_ramp();
    tracking_button_configuration_is_validated();
    chassis_only_has_no_tracking_publisher();
    stale_current_override_is_rejected();
    std::cout << "remote disconnect protection tests passed\n";
  } catch (const std::exception & error) {
    std::cerr << "remote disconnect protection: " << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
