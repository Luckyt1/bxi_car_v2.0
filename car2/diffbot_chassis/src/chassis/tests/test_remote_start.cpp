#include "chassis/control/remote_control.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <linux/joystick.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{
using namespace std::chrono_literals;
using GetParameters = rcl_interfaces::srv::GetParameters;
using SetParameters = rcl_interfaces::srv::SetParametersAtomically;
using Stage = chassis::RemoteControlSession::Stage;

std::string unique_suffix()
{
  return std::to_string(::getpid()) + "_" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
}

class TempJoystick
{
public:
  explicit TempJoystick(const std::string & suffix)
  : dir_("/tmp/remote-start-" + suffix),
    fifo_(dir_ + "/js0")
  {
    assert(::mkdir(dir_.c_str(), 0700) == 0);
    assert(::mkfifo(fifo_.c_str(), 0600) == 0);
    reopen();
  }

  ~TempJoystick()
  {
    close_writer();
    ::unlink(fifo_.c_str());
    ::rmdir(dir_.c_str());
  }

  const std::string & path() const {return fifo_;}

  void reopen()
  {
    close_writer();
    fd_ = ::open(fifo_.c_str(), O_RDWR | O_NONBLOCK);
    assert(fd_ >= 0);
  }

  void close_writer()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void axis(uint8_t number, int16_t value, bool initial = false)
  {
    event(JS_EVENT_AXIS, number, value, initial);
  }

  void button(uint8_t number, bool pressed, bool initial = false)
  {
    event(JS_EVENT_BUTTON, number, pressed ? 1 : 0, initial);
  }

private:
  void event(uint8_t type, uint8_t number, int16_t value, bool initial)
  {
    assert(fd_ >= 0);
    js_event event{};
    event.time = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    event.value = value;
    event.type = type | (initial ? JS_EVENT_INIT : 0);
    event.number = number;
    assert(::write(fd_, &event, sizeof(event)) == static_cast<ssize_t>(sizeof(event)));
  }

  std::string dir_;
  std::string fifo_;
  int fd_{-1};
};

class FakeChassis
{
public:
  FakeChassis(
    const rclcpp::Context::SharedPtr & context,
    const std::string & suffix,
    const std::string & parameter_node,
    const std::string & cmd_topic)
  {
    rclcpp::NodeOptions options;
    options.context(context);
    node = std::make_shared<rclcpp::Node>("fake_remote_start_" + suffix, options);
    values = {
      {"can3_motor_1_angle_deg", 0.0},
      {"can3_motor_2_angle_deg", 0.0},
      {"can3_motor_3_angle_deg", 0.0},
    };
    get_service = node->create_service<GetParameters>(
      parameter_node + "/get_parameters",
      [this](
        const std::shared_ptr<GetParameters::Request> request,
        std::shared_ptr<GetParameters::Response> response) {
        for (const auto & name : request->names) {
          get_calls.push_back(name);
          rcl_interfaces::msg::ParameterValue value;
          const auto match = values.find(name);
          if (match == values.end()) {
            value.type = rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;
          } else {
            value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
            value.double_value = match->second;
          }
          response->values.push_back(value);
        }
      });
    set_service = node->create_service<SetParameters>(
      parameter_node + "/set_parameters_atomically",
      [this](
        const std::shared_ptr<SetParameters::Request> request,
        std::shared_ptr<SetParameters::Response> response) {
        response->result.successful = true;
        for (const auto & parameter : request->parameters) {
          values[parameter.name] = parameter.value.double_value;
          set_calls.emplace_back(parameter.name, parameter.value.double_value);
        }
      });
    twist_subscription = node->create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic, 10,
      [this](const geometry_msgs::msg::Twist & message) {
        twists.push_back(message);
      });
  }

  bool has_forward_twist() const
  {
    for (const auto & twist : twists) {
      if (twist.linear.x > 0.0) {
        return true;
      }
    }
    return false;
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Service<GetParameters>::SharedPtr get_service;
  rclcpp::Service<SetParameters>::SharedPtr set_service;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription;
  std::map<std::string, double> values;
  std::vector<std::string> get_calls;
  std::vector<std::pair<std::string, double>> set_calls;
  std::vector<geometry_msgs::msg::Twist> twists;
};

template<typename Predicate>
bool spin_until(
  rclcpp::executors::SingleThreadedExecutor & executor,
  Predicate predicate,
  std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(5ms);
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  executor.spin_some(5ms);
  return predicate();
}

void spin_for(
  rclcpp::executors::SingleThreadedExecutor & executor,
  std::chrono::milliseconds duration)
{
  const bool reached = spin_until(executor, [] {return false;}, duration);
  assert(!reached);
  (void)reached;
}

class Harness
{
public:
  explicit Harness(
    std::shared_ptr<chassis::RemoteControlSession> remote_session =
    std::make_shared<chassis::RemoteControlSession>(),
    bool start_chassis = true,
    int start_button = 14)
  : suffix(unique_suffix()),
    parameter_node("/fake_can3_" + suffix),
    cmd_topic("/remote_start_cmd_vel_" + suffix),
    context(std::make_shared<rclcpp::Context>()),
    joystick(suffix),
    session(std::move(remote_session))
  {
    const char * argv[] = {"test_remote_start"};
    context->init(1, argv);
    fake = std::make_unique<FakeChassis>(context, suffix, parameter_node, cmd_topic);

    rclcpp::NodeOptions remote_options;
    remote_options.context(context);
    remote_options.parameter_overrides(
      {
        rclcpp::Parameter("device_path", joystick.path()),
        rclcpp::Parameter("cmd_vel_topic", cmd_topic),
        rclcpp::Parameter("can3_node", parameter_node),
        rclcpp::Parameter("axis_angular", 6),
        rclcpp::Parameter("timeout_sec", 0.0),
        rclcpp::Parameter("joystick_low_pass_time_constant_s", 0.0),
        rclcpp::Parameter("start_button", start_button),
      });
    if (session) {
      remote_options.append_parameter_override("start_chassis", start_chassis);
    }
    remote = chassis::make_remote_control(remote_options, session);

    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor->add_node(fake->node);
    executor->add_node(remote);
  }

  ~Harness()
  {
    if (executor) {
      try {
        if (remote) {
          executor->remove_node(remote);
        }
        if (fake && fake->node) {
          executor->remove_node(fake->node);
        }
      } catch (const std::exception &) {
      }
    }
    remote.reset();
    fake.reset();
    if (context && context->is_valid()) {
      context->shutdown("test complete");
    }
  }

  void neutral_startup(bool start_pressed = false, bool emergency_pressed = false)
  {
    joystick.button(1, false, true);
    joystick.button(3, false, true);
    joystick.button(11, emergency_pressed, true);
    joystick.button(14, start_pressed, true);
    joystick.axis(0, 0, true);
    joystick.axis(3, 0, true);
    joystick.axis(6, 0, true);
    joystick.axis(7, 0, true);
    spin_for(*executor, 250ms);
  }

  void request_start()
  {
    joystick.button(14, false);
    spin_for(*executor, 80ms);
    joystick.button(14, true);
    assert(
      spin_until(
        *executor,
        [this] {
          return session && session->stage == Stage::requested;
        },
        1s));
  }

  void drive_forward()
  {
    joystick.axis(3, -32767);
  }

  void can3_step_up()
  {
    joystick.axis(7, -32767);
  }

  std::string suffix;
  std::string parameter_node;
  std::string cmd_topic;
  rclcpp::Context::SharedPtr context;
  TempJoystick joystick;
  std::shared_ptr<chassis::RemoteControlSession> session;
  std::unique_ptr<FakeChassis> fake;
  rclcpp::Node::SharedPtr remote;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
};

void start_button_requires_release_and_known_emergency_release()
{
  Harness held;
  held.neutral_startup(true, false);
  held.joystick.button(14, true);
  spin_for(*held.executor, 250ms);
  assert(held.session->stage == Stage::waiting);
  held.request_start();

  Harness unknown_emergency;
  unknown_emergency.joystick.button(1, false, true);
  unknown_emergency.joystick.button(3, false, true);
  unknown_emergency.joystick.button(14, false, true);
  unknown_emergency.joystick.axis(0, 0, true);
  unknown_emergency.joystick.axis(3, 0, true);
  unknown_emergency.joystick.axis(6, 0, true);
  unknown_emergency.joystick.axis(7, 0, true);
  spin_for(*unknown_emergency.executor, 250ms);
  unknown_emergency.joystick.button(14, true);
  spin_for(*unknown_emergency.executor, 250ms);
  assert(unknown_emergency.session->stage == Stage::waiting);
  unknown_emergency.joystick.button(11, false);
  unknown_emergency.request_start();
}

void waiting_and_starting_gate_motion_and_can3_until_running()
{
  Harness harness;
  harness.neutral_startup();
  harness.drive_forward();
  harness.can3_step_up();
  spin_for(*harness.executor, 350ms);
  assert(harness.session->stage == Stage::waiting);
  assert(harness.fake->twists.empty());
  assert(harness.fake->get_calls.empty());
  assert(harness.fake->set_calls.empty());

  harness.request_start();
  harness.session->stage = Stage::starting;
  harness.joystick.axis(3, 0);
  harness.joystick.axis(7, 0);
  spin_for(*harness.executor, 100ms);
  harness.drive_forward();
  harness.can3_step_up();
  spin_for(*harness.executor, 350ms);
  assert(harness.fake->twists.empty());
  assert(harness.fake->get_calls.empty());
  assert(harness.fake->set_calls.empty());

  harness.session->stage = Stage::running;
  // Held controls from calibration must remain locked until they return to neutral.
  spin_for(*harness.executor, 200ms);
  assert(!harness.fake->has_forward_twist());
  assert(harness.fake->get_calls.empty());
  assert(harness.fake->set_calls.empty());
  harness.joystick.axis(3, 0);
  harness.joystick.axis(7, 0);
  spin_for(*harness.executor, 200ms);
  harness.drive_forward();
  assert(
    spin_until(
      *harness.executor,
      [&harness] {
        return harness.fake->has_forward_twist();
      },
      1s));
  harness.can3_step_up();
  assert(
    spin_until(
      *harness.executor,
      [&harness] {
        return !harness.fake->set_calls.empty();
      },
      2s));
  assert(harness.fake->get_calls == std::vector<std::string>{"can3_motor_1_angle_deg"});
  assert(harness.fake->set_calls.size() == 1);
  assert(harness.fake->set_calls.front().first == "can3_motor_1_angle_deg");
  assert(std::abs(harness.fake->set_calls.front().second - 1.0) < 1e-9);
}

void repeated_start_does_not_retrigger_after_main_takes_ownership()
{
  Harness harness;
  harness.neutral_startup();
  harness.request_start();

  harness.session->stage = Stage::starting;
  harness.joystick.button(14, false);
  spin_for(*harness.executor, 80ms);
  harness.joystick.button(14, true);
  spin_for(*harness.executor, 250ms);
  assert(harness.session->stage == Stage::starting);

  harness.session->stage = Stage::running;
  harness.joystick.button(14, false);
  spin_for(*harness.executor, 80ms);
  harness.joystick.button(14, true);
  spin_for(*harness.executor, 250ms);
  assert(harness.session->stage == Stage::running);
}

void remote_only_modes_bypass_start_gate()
{
  {
    Harness no_session(nullptr, true);
    no_session.neutral_startup();
    no_session.drive_forward();
    assert(
      spin_until(
        *no_session.executor,
        [&no_session] {
          return no_session.fake->has_forward_twist();
        },
        1s));
  }
  {
    auto session = std::make_shared<chassis::RemoteControlSession>();
    Harness disabled(session, false);
    disabled.neutral_startup();
    disabled.drive_forward();
    assert(
      spin_until(
        *disabled.executor,
        [&disabled] {
          return disabled.fake->has_forward_twist();
        },
        1s));
  }
}

void emergency_stops_waiting_context()
{
  Harness harness;
  harness.neutral_startup();
  harness.joystick.button(11, true);
  assert(
    spin_until(
      *harness.executor,
      [&harness] {
        return !harness.context->is_valid();
      },
      1s));
}

void waiting_disconnect_resets_start_release()
{
  Harness harness;
  harness.neutral_startup();
  harness.request_start();
  harness.joystick.close_writer();
  assert(
    spin_until(
      *harness.executor,
      [&harness] {
        return harness.session->stage == Stage::waiting;
      },
      1s));

  harness.joystick.reopen();
  harness.joystick.button(11, false, true);
  harness.joystick.button(14, true, true);
  harness.joystick.axis(0, 0, true);
  harness.joystick.axis(3, 0, true);
  harness.joystick.axis(6, 0, true);
  harness.joystick.axis(7, 0, true);
  spin_for(*harness.executor, 250ms);
  harness.joystick.button(14, true);
  spin_for(*harness.executor, 250ms);
  assert(harness.session->stage == Stage::waiting);
  harness.request_start();
}

void starting_disconnect_shuts_down_context()
{
  Harness harness;
  harness.neutral_startup();
  harness.request_start();
  harness.session->stage = Stage::starting;
  harness.joystick.close_writer();
  assert(
    spin_until(
      *harness.executor,
      [&harness] {
        return !harness.context->is_valid();
      },
      1s));
}

void invalid_start_parameters_throw()
{
  const auto construct_throws = [](int start_button) {
      const auto suffix = unique_suffix();
      auto context = std::make_shared<rclcpp::Context>();
      const char * argv[] = {"test_remote_start_invalid"};
      context->init(1, argv);
      TempJoystick joystick(suffix);
      rclcpp::NodeOptions remote_options;
      remote_options.context(context);
      remote_options.parameter_overrides(
      {
        rclcpp::Parameter("device_path", joystick.path()),
        rclcpp::Parameter("start_chassis", true),
        rclcpp::Parameter("start_button", start_button),
      });
      bool threw = false;
      try {
        auto session = std::make_shared<chassis::RemoteControlSession>();
        auto remote = chassis::make_remote_control(remote_options, session);
        (void)remote;
      } catch (const std::invalid_argument &) {
        threw = true;
      }
      if (context->is_valid()) {
        context->shutdown("invalid parameter test complete");
      }
      return threw;
    };

  assert(construct_throws(16));
  assert(construct_throws(11));
}

}  // namespace

int main()
{
  start_button_requires_release_and_known_emergency_release();
  waiting_and_starting_gate_motion_and_can3_until_running();
  repeated_start_does_not_retrigger_after_main_takes_ownership();
  remote_only_modes_bypass_start_gate();
  emergency_stops_waiting_context();
  waiting_disconnect_resets_start_release();
  starting_disconnect_shuts_down_context();
  invalid_start_parameters_throw();
  return 0;
}
