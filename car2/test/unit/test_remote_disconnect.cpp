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
  RemoteFixture(bool connected, bool waiting, double timeout_s = 0.0)
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
      rclcpp::Parameter("invert_linear_axis", false),
      rclcpp::Parameter("invert_lateral_axis", false),
      rclcpp::Parameter("invert_angular_axis", false),
      rclcpp::Parameter("max_accel", 0.2),
      rclcpp::Parameter("max_decel", 0.01)};
    if (timeout_s > 0.0) {parameters.emplace_back("timeout_sec", timeout_s);}
    options.parameter_overrides(parameters);
    if (waiting) {session_ = std::make_shared<chassis::RemoteControlSession>();}
    link_available_ = std::make_shared<std::atomic<bool>>(true);
    link_monitor_ = std::make_shared<chassis::JoystickLinkMonitor>(
      [available = link_available_] {return available->load();});
    remote_ = chassis::make_remote_control(options, session_, false, "", link_monitor_);
    observer_ = std::make_shared<rclcpp::Node>(
      "remote_disconnect_test", rclcpp::NodeOptions().use_intra_process_comms(true));
    subscription_ = observer_->create_subscription<Twist>(
      "/remote_disconnect_test/cmd_vel", 10,
      [this](Twist::ConstSharedPtr command) {commands.push_back(*command);});
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
    std::cout << "remote disconnect protection tests passed\n";
  } catch (const std::exception & error) {
    std::cerr << "remote disconnect protection: " << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
