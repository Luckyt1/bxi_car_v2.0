#include "chassis/control/remote_control.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
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
#include <map>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{
using namespace std::chrono_literals;
using GetParameters = rcl_interfaces::srv::GetParameters;
using SetParameters = rcl_interfaces::srv::SetParametersAtomically;

std::string unique_suffix()
{
  return std::to_string(::getpid()) + "_" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
}

class TempJoystick
{
public:
  explicit TempJoystick(const std::string & suffix)
  : dir_("/tmp/remote-composition-" + suffix),
    fifo_(dir_ + "/js0")
  {
    assert(::mkdir(dir_.c_str(), 0700) == 0);
    assert(::mkfifo(fifo_.c_str(), 0600) == 0);
    fd_ = ::open(fifo_.c_str(), O_RDWR | O_NONBLOCK);
    assert(fd_ >= 0);
  }

  ~TempJoystick()
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
    ::unlink(fifo_.c_str());
    ::rmdir(dir_.c_str());
  }

  const std::string & path() const {return fifo_;}

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
    node = std::make_shared<rclcpp::Node>("fake_chassis_" + suffix, options);
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
          assert(
            parameter.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);
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

}  // namespace

int main()
{
  const auto suffix = unique_suffix();
  TempJoystick joystick(suffix);
  const std::string parameter_node = "/fake_can3_" + suffix;
  const std::string cmd_topic = "/remote_composition_cmd_vel_" + suffix;

  auto context = std::make_shared<rclcpp::Context>();
  const char * argv[] = {"test_remote_composition"};
  context->init(1, argv);

  FakeChassis fake(context, suffix, parameter_node, cmd_topic);

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
  });
  auto remote = chassis::make_remote_control(remote_options);

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(fake.node);
  executor.add_node(remote);

  joystick.button(1, false, true);
  joystick.button(3, false, true);
  joystick.axis(0, 0, true);
  joystick.axis(3, 0, true);
  joystick.axis(6, 0, true);
  joystick.axis(7, 0, true);
  spin_for(executor, 250ms);

  joystick.axis(3, -32767);
  assert(
    spin_until(
      executor,
      [&fake] {
        for (const auto & twist : fake.twists) {
          if (twist.linear.x > 0.0) {
            return true;
          }
        }
        return false;
      },
      2s));

  joystick.axis(7, -32767);
  assert(
    spin_until(
      executor,
      [&fake] {
        return !fake.set_calls.empty();
      },
      2s));

  assert(fake.get_calls.size() == 1);
  assert(fake.get_calls.front() == "can3_motor_1_angle_deg");
  assert(fake.set_calls.size() == 1);
  assert(fake.set_calls.front().first == "can3_motor_1_angle_deg");
  assert(std::abs(fake.set_calls.front().second - 1.0) < 1e-9);
  assert(std::abs(fake.values["can3_motor_1_angle_deg"] - 1.0) < 1e-9);
  assert(context->is_valid());

  // An emergency stop must shut down the caller's context, including during startup polling.
  joystick.button(11, true);
  assert(spin_until(executor, [&context] {return !context->is_valid();}, 2s));

  executor.remove_node(remote);
  executor.remove_node(fake.node);
  remote.reset();
  fake.node.reset();
  assert(!context->is_valid());
  return 0;
}
