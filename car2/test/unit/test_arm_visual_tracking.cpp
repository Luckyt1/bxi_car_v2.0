#include "example/arm_visual_tracking.h"
#include "example/app_arm_control.h"
#include "rcl/time.h"

#include <array>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>

namespace
{
using namespace std::chrono_literals;
using iswv::canopen::ObjectAddress;
using iswv::canopen::SdoResponse;
using Clock = chassis::ArmVisualTracking::Clock;
constexpr double kPulses = 3600000.0;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

class CoeNode final : public chassis::ethercat::Node
{
public:
  explicit CoeNode(std::uint16_t id) : id_(id)
  {
    put<std::uint16_t>(0x6041, 0x40);
    put<std::uint8_t>(0x2100, 1);
    put<std::int8_t>(0x6060, 0);
    put<std::int8_t>(0x6061, 0);
    put<std::int8_t>(0x2707, 1);
    put<std::int32_t>(0x6064, 0);
    put<std::int32_t>(0x606C, 0);
    put<std::int32_t>(0x60FF, 0);
    put<std::int16_t>(0x6071, 0);
    put<std::int16_t>(0x6078, 0);
    put<std::int16_t>(0x60B2, 0);
    put<std::int32_t>(0x2025, static_cast<std::int32_t>(kPulses));
    put<std::uint16_t>(0x26A2, 1);
    put<std::uint16_t>(0x26A3, 1);
  }

  std::uint16_t id() const noexcept override {return id_;}

  template<typename T>
  void put(std::uint16_t index, T value)
  {
    const auto bytes = iswv::canopen::encode_little_endian(value);
    values_[{index, 0}] = {bytes.begin(), bytes.end()};
  }

  template<typename T>
  T get(std::uint16_t index)
  {
    const auto result = read(iswv::canopen::Object<T>{{index, 0}, "fake object"});
    require(bool(result), "fake object must exist");
    return result.value();
  }

  iswv::Result<SdoResponse> upload_blocking(
    ObjectAddress object, iswv::canopen::SdoOptions) override
  {
    ++reads;
    const auto found = values_.find(object);
    if (found == values_.end()) {
      return iswv::Result<SdoResponse>::failure(
        iswv::make_sdo_abort(0x06020000, "unknown test object"));
    }
    SdoResponse response;
    response.object = object;
    response.size = static_cast<std::uint8_t>(found->second.size());
    std::copy(found->second.begin(), found->second.end(), response.data.begin());
    return iswv::Result<SdoResponse>::success(response);
  }

  iswv::Result<void> download_blocking(
    ObjectAddress object, const std::uint8_t * data, std::size_t size,
    iswv::canopen::SdoOptions) override
  {
    ++writes;
    std::uint32_t raw = 0;
    for (std::size_t i = 0; i < size; ++i) {
      raw |= static_cast<std::uint32_t>(data[i]) << (8U * i);
    }
    if (object.index == 0x6040 && (raw & 0x0100U) && fail_halt) {
      return iswv::Result<void>::failure(
        iswv::make_sdo_abort(0x08000000, "injected halt failure"));
    }
    values_[object] = {data, data + size};
    if (object.index == 0x6060) {put<std::int8_t>(0x6061, static_cast<std::int8_t>(raw));}
    if (object.index == 0x6040) {
      std::uint16_t status = 0x40;
      switch (raw & 0x0f) {
        case 0x0b: status = 0x07; break;
        case 0x06: status = 0x21; break;
        case 0x07: status = 0x23; break;
        case 0x0f: status = (raw & 0x10U) ? 0x1027 : 0x27; break;
        default: break;
      }
      if (raw & 0x10U) {++triggers;}
      put<std::uint16_t>(0x6041, status);
    }
    return iswv::Result<void>::success();
  }

  unsigned triggers{0};
  unsigned reads{0};
  unsigned writes{0};
  bool fail_halt{false};

private:
  std::uint16_t id_;
  std::map<ObjectAddress, std::vector<std::uint8_t>> values_;
};

struct Fixture
{
  explicit Fixture(bool controlled_ros_time = false, bool fourth_axis = true)
  {
    for (std::size_t i = 0; i < motors.size(); ++i) {
      motors[i] = std::make_shared<CoeNode>(static_cast<std::uint16_t>(i + 1));
    }
    controller = std::make_unique<chassis::ArmController>(
      std::array<std::shared_ptr<chassis::ethercat::Node>, 3>{motors[0], motors[1], motors[2]},
      [this] {power_off = true;}, fourth_axis ? motors[3] : nullptr);
    controller->initialize();
    node = std::make_shared<rclcpp::Node>(
      "arm_visual_tracking_test", rclcpp::NodeOptions().use_intra_process_comms(true));
    if (controlled_ros_time) {
      require(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()) == RCL_RET_OK,
        "test ROS clock override must enable");
      ros_time_ns = 10000000000LL;
      require(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), *ros_time_ns) ==
        RCL_RET_OK, "test ROS time must initialize");
    }
    chassis::declare_arm_pose_parameter(*node);
    chassis::declare_arm_tracking_parameter(*node);
    tracker = std::make_unique<chassis::ArmVisualTracking>(*node, *controller);
    now = Clock::now() + 20ms;
    initial = controller->actual_pose();
  }

  void tick(std::chrono::milliseconds elapsed = 20ms, bool plan = true)
  {
    now += elapsed;
    if (ros_time_ns) {
      *ros_time_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
      require(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), *ros_time_ns) ==
        RCL_RET_OK, "test ROS time must advance");
    }
    tracker->before_update(now);
    controller->update(now);
    if (plan) {tracker->after_update();}
  }

  void enable()
  {
    require(node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", true)).successful,
      "enable must commit");
    tick();
  }

  void enable_x()
  {
    require(node->set_parameters_atomically({
        rclcpp::Parameter("arm_tracking_x_enabled", true),
        rclcpp::Parameter("arm_tracking_enabled", true)}).successful,
      "atomic X tracking enable must commit");
    tick();
  }

  void enable_goal()
  {
    require(node->set_parameters_atomically({
        rclcpp::Parameter("arm_tracking_x_enabled", true),
        rclcpp::Parameter("arm_tracking_goal_enabled", true),
        rclcpp::Parameter("arm_tracking_enabled", true)}).successful,
      "atomic fixed goal enable must commit");
    tick();
  }

  geometry_msgs::msg::PointStamped message(double z, double y = 0.5)
  {
    geometry_msgs::msg::PointStamped result;
    result.header.frame_id = "arm_shoulder";
    result.header.stamp = node->now();
    result.point.x = 1.0;  // Legacy Z-only sessions must ignore this X coordinate.
    result.point.y = y;
    result.point.z = z;
    return result;
  }

  void target(double z, double y = 0.5) {tracker->observe_target(message(z, y), now);}

  void target_xz(double x, double z, double y = 0.5)
  {
    auto point = message(z, y);
    point.point.x = x;
    tracker->observe_target(point, now);
  }

  geometry_msgs::msg::PointStamped goal_message(double x, double z)
  {
    auto point = message(z, 0.0);
    point.point.x = x;
    return point;
  }

  void goal(double x, double z) {tracker->observe_goal(goal_message(x, z), now);}

  void start_target()
  {
    enable();
    target(initial.z + 0.05);
    tick(100ms);
    require(controller->position_command_pending(), "fresh target must queue the complete endpoint");
  }

  void finish_handshake()
  {
    for (unsigned i = 0; i < 8 && controller->position_command_pending(); ++i) {
      tick(20ms, false);
    }
    require(!controller->position_command_pending(), "fake PP handshake must finish");
  }

  bool halted()
  {
    return std::all_of(motors.begin(), motors.begin() + 3, [](const auto & motor) {
      return (motor->template get<std::uint16_t>(0x6040) & 0x0100U) != 0;
    });
  }

  arm::JointAngles commanded_joints()
  {
    arm::JointAngles angles{};
    for (std::size_t i = 0; i < angles.size(); ++i) {
      angles[i] = motors[i]->get<std::int32_t>(0x607A) * 2.0 * arm::kPi / kPulses;
    }
    return angles;
  }

  arm::Pose commanded_pose()
  {
    return arm::Kinematics(arm::Configuration{}).forward(commanded_joints());
  }

  void measure_joints(const arm::JointAngles & joints)
  {
    for (std::size_t i = 0; i < joints.size(); ++i) {
      motors[i]->put<std::int32_t>(0x6064, static_cast<std::int32_t>(std::llround(
          joints[i] * kPulses / (2.0 * arm::kPi))));
    }
  }

  void measure(const arm::Pose & pose)
  {
    const auto result = arm::Kinematics(arm::Configuration{}).inverse(pose, arm::JointAngles{0, 0, 0});
    require(result.status == arm::InverseStatus::Success, "test measured pose must be feasible");
    measure_joints(result.solutions.front().joints);
  }

  bool power_off{false};
  std::array<std::shared_ptr<CoeNode>, 4> motors;
  std::unique_ptr<chassis::ArmController> controller;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<chassis::ArmVisualTracking> tracker;
  Clock::time_point now;
  std::optional<std::int64_t> ros_time_ns;
  arm::Pose initial;
};

void lateral_direction_limits_and_independent_deadband()
{
  Fixture f(true);
  require(!f.tracker->owns_chassis(), "disabled tracking must not own the chassis");
  require(f.tracker->accept_manual_chassis_command(0.1, 0.0, 0.0, f.now),
    "ordinary manual chassis commands must be accepted before tracking");
  require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", true)).successful,
    "enable request must commit");
  require(f.tracker->owns_chassis(), "an enable awaiting the control tick must already own chassis");
  require(f.tracker->lateral_velocity(f.now) == 0.0,
    "activation without settled feedback or a target must stay stopped");
  f.tick();
  require(f.tracker->lateral_velocity(f.now) == 0.0, "settled feedback alone cannot move chassis");

  const std::array<std::pair<double, double>, 9> cases{{
    {0.004, 0.008}, {-0.004, -0.008}, {0.5, 0.020}, {-0.5, -0.020},
    {0.002, 0.0}, {-0.002, 0.0}, {0.0, 0.0}, {1e300, 0.020}, {-1e300, -0.020}}};
  for (const auto & [error, expected] : cases) {
    f.tick();
    f.target(f.initial.z, error);
    f.tracker->after_update();
    require(std::abs(f.tracker->lateral_velocity(f.now) - expected) < 1e-12,
      "left/right errors must map to bounded signed vy, with a separate lateral deadband");
    require(!f.controller->position_command_pending(),
      "lateral correction must work with height centered without moving planar joints");
    require(!f.tracker->accept_manual_chassis_command(0.0, 0.0, 0.0, f.now),
      "manual neutral heartbeat must not take visual chassis ownership");
    require(std::abs(f.tracker->lateral_velocity(f.now) - expected) < 1e-12,
      "manual neutral heartbeat must preserve the visual lateral command");
  }
}

void lateral_waits_for_halt_and_exit_requires_neutral()
{
  Fixture f(true);
  f.motors[0]->put<std::int8_t>(0x2707, 0);
  f.motors[0]->put<std::int32_t>(0x606C, 6000);
  f.enable();
  f.tick();
  f.target(f.initial.z, 0.01);
  require(f.tracker->lateral_velocity(f.now) == 0.0,
    "chassis must wait for planar standstill during activation");
  f.motors[0]->put<std::int8_t>(0x2707, 1);
  f.motors[0]->put<std::int32_t>(0x606C, 0);
  f.tick();
  require(f.tracker->lateral_velocity(f.now) == 0.020,
    "valid horizontal input may move after standstill and baseline acquisition");
  require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", false)).successful,
    "visual disable must commit");
  require(f.tracker->owns_chassis(), "pending disable cannot expose the last visual velocity");
  require(f.tracker->lateral_velocity(f.now) == 0.0, "disable must zero lateral motion immediately");
  require(!f.tracker->accept_manual_chassis_command(0.0, 0.0, 0.0, f.now),
    "neutral command cannot release ownership before arm handover settles");
  f.tick();
  require(f.tracker->owns_chassis(), "handover still requires a fresh manual neutral");
  require(!f.tracker->accept_manual_chassis_command(0.0, 0.2, 0.0, f.now),
    "a held manual command must not replay after visual exit");
  require(f.tracker->accept_manual_chassis_command(0.0, 0.0, 0.0, f.now),
    "post-handover neutral must release manual chassis interlock");
  require(!f.tracker->owns_chassis(), "manual neutral must release chassis ownership without deadlock");
  require(f.tracker->accept_manual_chassis_command(0.0, -0.2, 0.0, f.now),
    "fresh manual motion may resume after neutral");
}

void manual_motion_and_watchdog_latch_both_visual_axes()
{
  const std::array<std::array<double, 3>, 5> commands{{
    {0.1, 0.0, 0.0}, {0.0, -0.1, 0.0}, {0.0, 0.0, 0.1},
    {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
    {0.0, std::numeric_limits<double>::infinity(), 0.0}}};
  for (std::size_t kind = 0; kind <= commands.size(); ++kind) {
    Fixture f(true);
    f.enable();
    f.tick();
    f.target(f.initial.z + 0.05, 0.01);
    f.tick(100ms);
    require(f.controller->position_command_pending() && f.tracker->lateral_velocity(f.now) > 0,
      "fixture must have both a staged height move and a lateral command");
    if (kind < commands.size()) {
      const auto & command = commands[kind];
      require(!f.tracker->accept_manual_chassis_command(
          command[0], command[1], command[2], f.now),
        "manual motion or invalid input must interrupt tracking before taking control");
    } else {
      f.tracker->cancel_tracking("manual command stream timed out", f.now);
    }
    require(f.tracker->lateral_velocity(f.now) == 0.0 && f.halted() &&
      !f.controller->position_command_pending(), "interruption must stop both visual axes");
    f.tick();
    f.target(f.initial.z + 0.05, -0.01);
    f.tick();
    require(f.tracker->lateral_velocity(f.now) == 0.0 &&
      !f.controller->position_command_pending(), "fresh targets cannot bypass an interruption latch");
    require(!f.tracker->accept_manual_chassis_command(0.0, 0.0, 0.0, f.now),
      "neutral cannot release a still-enabled tracking latch");
    require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", false)).successful,
      "operator disable must commit");
    f.tick();
    require(!f.tracker->accept_manual_chassis_command(0.1, 0.0, 0.0, f.now),
      "manual input must still return to neutral after interrupted tracking exits");
    require(f.tracker->accept_manual_chassis_command(0.0, 0.0, 0.0, f.now),
      "neutral must restore manual ownership after interruption");
    f.enable();
    f.tick();
    f.target(f.initial.z, -0.01);
    require(f.tracker->lateral_velocity(f.now) == -0.020,
      "explicit disable and re-enable must permit a newly acquired safe session");
  }
}

void explicit_stop_clears_lateral_and_requires_a_new_capture()
{
  Fixture f(true);
  f.enable();
  f.tick();
  const auto old = f.message(f.initial.z, 0.01);
  f.tracker->observe_target(old, f.now);
  require(f.tracker->lateral_velocity(f.now) > 0, "fixture must command lateral motion");
  f.tracker->stop_tracking(f.now);
  require(f.tracker->lateral_velocity(f.now) == 0.0, "explicit stop must clear lateral velocity");
  f.tracker->observe_target(old, f.now);
  require(f.tracker->lateral_velocity(f.now) == 0.0,
    "a capture predating stop cannot restart lateral movement");
  f.tick();
  f.target(f.initial.z, -0.01);
  require(f.tracker->lateral_velocity(f.now) < 0,
    "after standstill an explicitly newer capture may resume an ordinary stop");
}

void lateral_motion_continues_until_disabled_and_recovers_after_loss()
{
  Fixture f(true);
  f.enable();
  f.tick();
  f.target(f.initial.z, 0.01);
  require(f.tracker->lateral_velocity(f.now) > 0, "fixture must begin lateral motion");
  for (unsigned i = 0; i < 600; ++i) {
    f.tick(100ms);
    f.target(f.initial.z, 0.01);
    require(f.tracker->lateral_velocity(f.now) > 0,
      "fresh targets must sustain lateral motion throughout a sixty-second session");
  }

  f.tick(20000ms);
  require(f.tracker->lateral_velocity(f.now) == 0.0 && f.halted(),
    "missing targets must still halt both visual axes without a session time limit");
  f.tick();
  f.target(f.initial.z, -0.01);
  require(f.tracker->lateral_velocity(f.now) < 0,
    "a fresh capture after a long target loss must resume without a mode toggle");
  for (unsigned i = 0; i < 150; ++i) {
    f.tick(100ms);
    f.target(f.initial.z, -0.01);
    require(f.tracker->lateral_velocity(f.now) < 0,
      "resumed tracking must also continue beyond ten seconds");
  }

  require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", false)).successful,
    "operator disable must commit after a long tracking session");
  f.tick();
  require(f.tracker->lateral_velocity(f.now) == 0.0 && f.halted(),
    "disabling tracking must halt both visual axes immediately");
  f.tick();
  f.target(f.initial.z + 0.05, 0.01);
  f.tick(100ms);
  require(f.tracker->lateral_velocity(f.now) == 0.0 &&
    !f.controller->position_command_pending(), "new images cannot restart disabled tracking");
  f.enable();
  f.tick();
  f.target(f.initial.z, 0.01);
  require(f.tracker->lateral_velocity(f.now) > 0,
    "a new activation and fresh target must resume tracking");
}

void defaults_and_startup_guard()
{
  Fixture f;
  require(!f.node->get_parameter("arm_tracking_enabled").as_bool(), "tracking defaults off");
  require(!f.node->get_parameter("arm_tracking_x_enabled").as_bool(), "X tracking defaults off");
  require(!f.node->get_parameter("arm_tracking_goal_enabled").as_bool(), "fixed goal defaults off");
  f.target(f.initial.z + 0.05);
  f.tick();
  require(!f.controller->position_command_pending(), "disabled vision must not move");
  for (const char * parameter : {"arm_tracking_enabled", "arm_tracking_x_enabled",
      "arm_tracking_goal_enabled"})
  {
    auto startup = std::make_shared<rclcpp::Node>("tracking_bad_startup",
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter(parameter, true)}));
    bool rejected = false;
    try {chassis::declare_arm_tracking_parameter(*startup);} catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "startup override must not enable any tracking mode");
  }
}

void x_mode_changes_require_disabled_tracking_or_atomic_disable()
{
  Fixture f(true);
  require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_x_enabled", true)).successful,
    "X tracking can be configured while disabled");
  require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_x_enabled", false)).successful,
    "disabled X tracking configuration can be cleared");
  require(f.node->set_parameters_atomically({
      rclcpp::Parameter("arm_tracking_x_enabled", true),
      rclcpp::Parameter("arm_tracking_enabled", true)}).successful,
    "X tracking and visual ownership can be enabled atomically");
  require(!f.node->set_parameter(rclcpp::Parameter("arm_tracking_x_enabled", false)).successful,
    "pending activation must already protect the selected X mode");
  f.tick();
  require(!f.node->set_parameters_atomically({
      rclcpp::Parameter("arm_tracking_x_enabled", false),
      rclcpp::Parameter("arm_tracking_enabled", true)}).successful,
    "an enabled session cannot change X mode by repeating enable");
  require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_x_enabled", true)).successful,
    "writing the unchanged X mode is harmless");
  require(f.node->set_parameters_atomically({
      rclcpp::Parameter("arm_tracking_x_enabled", false),
      rclcpp::Parameter("arm_tracking_enabled", false)}).successful,
    "an atomic disable may also clear X tracking");
  require(!f.node->set_parameter(rclcpp::Parameter("arm_tracking_x_enabled", true)).successful,
    "X reconfiguration must wait for a pending disable to reach the hardware owner");
  f.tick();
  require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_x_enabled", true)).successful,
    "X tracking may be configured again after disable is applied");
  const std::vector<double> manual{f.initial.x, f.initial.z + 0.01, f.initial.pitch};
  require(!f.node->set_parameters_atomically({
      rclcpp::Parameter("arm_tracking_x_enabled", false),
      rclcpp::Parameter("yiyou_arm_target", manual)}).successful,
    "X mode changes cannot smuggle a simultaneous manual target through arbitration");
}

void depth_x_moves_both_directions_and_keeps_z_and_pitch()
{
  for (int direction : {-1, 1}) {
    Fixture f(true);
    f.measure({0.35, 0.35, 0.0});
    f.enable_x();
    f.tick();
    const auto baseline = f.controller->actual_pose();
    // The absolute X target is not restricted to a 15 cm activation window.
    const double x = baseline.x + direction * 0.20;
    f.target_xz(x, baseline.z);
    f.tick(100ms);
    require(f.controller->position_command_pending(), "pure X error must queue the full target");
    f.target_xz(x, baseline.z);
    f.finish_handshake();
    const auto next = f.commanded_pose();
    require(std::abs(direction * (next.x - baseline.x) - 0.20) < 0.000002,
      "pure X tracking must submit the complete target in either direction");
    require(std::abs(next.z - baseline.z) < 0.000002 &&
      std::abs(next.pitch - baseline.pitch) < 0.000002,
      "pure X tracking must preserve height and activation pitch");
  }
}

void depth_diagonal_targets_are_complete_at_any_update_interval()
{
  for (const auto elapsed : {20ms, 100ms, 240ms}) {
    Fixture f(true);
    f.measure({0.35, 0.35, 0.0});
    f.enable_x();
    f.tick();
    const auto baseline = f.controller->actual_pose();
    f.target_xz(baseline.x + 0.03, baseline.z + 0.04);
    f.tick(elapsed);
    require(f.controller->position_command_pending(), "a diagonal target must queue in full");
    f.target_xz(baseline.x + 0.03, baseline.z + 0.04);
    f.finish_handshake();
    const auto next = f.commanded_pose();
    require(std::abs(next.x - baseline.x - 0.03) < 0.000002 &&
      std::abs(next.z - baseline.z - 0.04) < 0.000002 &&
      std::abs(next.pitch - baseline.pitch) < 0.000002,
      "elapsed time must not scale a valid X/Z endpoint or change activation pitch");
  }
  Fixture close(true);
  close.measure({0.35, 0.35, 0.0});
  close.enable_x();
  close.tick();
  const auto baseline = close.controller->actual_pose();
  close.target_xz(baseline.x + 0.0015, baseline.z + 0.002);
  close.tick();
  close.finish_handshake();
  const auto next = close.commanded_pose();
  require(std::abs(next.x - baseline.x - 0.0015) < 0.000002 &&
    std::abs(next.z - baseline.z - 0.002) < 0.000002,
    "a close diagonal target must be submitted exactly without overshoot");
}

void latest_depth_target_waits_only_for_the_previous_handshake()
{
  Fixture f(true);
  f.measure({0.35, 0.35, 0.0});
  f.enable_x();
  f.tick();
  const auto baseline = f.controller->actual_pose();
  f.target_xz(baseline.x + 0.05, baseline.z);
  f.tick();
  const auto triggers = f.motors[0]->triggers;
  f.target_xz(baseline.x + 0.06, baseline.z);
  f.tick();
  f.target_xz(baseline.x + 0.08, baseline.z);
  require(f.controller->position_command_pending() && f.motors[0]->triggers == triggers,
    "new captures must not overwrite an in-flight PP handshake");
  f.finish_handshake();
  require(std::abs(f.commanded_pose().x - baseline.x - 0.05) < 0.000002,
    "the initial complete endpoint must finish its handshake first");
  f.target_xz(baseline.x + 0.08, baseline.z);
  f.tracker->after_update();
  require(f.controller->position_command_pending(),
    "the latest target must queue without waiting for the previous physical arrival");
  f.finish_handshake();
  require(std::abs(f.commanded_pose().x - baseline.x - 0.08) < 0.000002 &&
    std::abs(f.controller->actual_pose().x - baseline.x) < 0.000002,
    "frozen feedback must not prevent replacing the endpoint with the newest capture");
  require(f.motors[0]->triggers == triggers + 2,
    "only the initial and latest endpoints may trigger; intermediate captures must be superseded");
}

void depth_x_reversal_halts_before_accepting_a_new_capture()
{
  for (bool accepted : {false, true}) {
    Fixture f(true);
    f.enable_x();
    f.tick();
    f.target_xz(f.initial.x + 0.05, f.initial.z);
    f.tick(100ms);
    if (accepted) {
      f.target_xz(f.initial.x + 0.05, f.initial.z);
      f.finish_handshake();
    }
    f.target_xz(f.initial.x - 0.05, f.initial.z);
    require(f.halted() && !f.controller->position_command_pending() &&
      f.tracker->lateral_velocity(f.now) == 0.0,
      "an X reversal must cancel an unaccepted or accepted target and lateral motion");
    f.tick();
    f.target_xz(f.initial.x - 0.05, f.initial.z);
    f.tick(100ms);
    f.target_xz(f.initial.x - 0.05, f.initial.z);
    f.finish_handshake();
    require(std::abs(f.commanded_pose().x - f.initial.x + 0.05) < 0.000002,
      "a fresh post-stop capture may submit the complete reverse X target");
  }
}

void unreachable_depth_target_is_rejected_without_partial_motion()
{
  for (double x : {0.6858, std::sqrt(0.7 * 0.7 - 0.14 * 0.14)}) {
    Fixture f(true);
    f.measure({x, 0.14, 0.0});
    f.enable_x();
    f.tick();
    const auto baseline = f.controller->actual_pose();
    const auto triggers = f.motors[0]->triggers;
    f.target_xz(baseline.x + 0.05, baseline.z + 0.05);
    f.tick();
    require(f.halted() && !f.controller->position_command_pending() &&
      f.motors[0]->triggers == triggers && f.tracker->lateral_velocity(f.now) == 0.0,
      "an unreachable complete endpoint must halt without substituting a smaller move");
    require(f.controller->fault_reason().empty() && !f.power_off,
      "unreachable vision input must remain an input rejection, not a hardware fault");
  }
}

void depth_tracking_stop_expiry_and_handover_halt_all_visual_motion()
{
  for (unsigned interruption = 0; interruption < 4; ++interruption) {
    Fixture f(true);
    f.enable_x();
    f.tick();
    f.target_xz(f.initial.x + 0.05, f.initial.z + 0.05);
    f.tick(100ms);
    require(f.controller->position_command_pending() && f.tracker->lateral_velocity(f.now) > 0,
      "fixture must command depth-based planar motion and chassis lateral motion");
    if (interruption == 0) {
      f.tracker->stop_tracking(f.now);
    } else if (interruption == 1) {
      f.tick(400ms);
    } else if (interruption == 2) {
      require(!f.tracker->accept_manual_chassis_command(0.1, 0.0, 0.0, f.now),
        "manual takeover must stop depth tracking before accepting manual velocity");
    } else {
      require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", false)).successful,
        "operator mode exit must commit");
      f.tick();
    }
    require(f.halted() && !f.controller->position_command_pending() &&
      f.tracker->lateral_velocity(f.now) == 0.0,
      "stop, stale target, manual takeover and mode exit must halt X/Z and chassis together");
  }
}

void goal_mode_requires_x_and_atomic_mode_transitions()
{
  Fixture f(true);
  require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_goal_enabled", true)).successful,
    "fixed goal can be configured while tracking is disabled");
  require(!f.node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", true)).successful,
    "fixed goal activation requires X tracking in the final parameter state");
  require(f.node->set_parameters_atomically({
      rclcpp::Parameter("arm_tracking_x_enabled", true),
      rclcpp::Parameter("arm_tracking_goal_enabled", true),
      rclcpp::Parameter("arm_tracking_enabled", true)}).successful,
    "fixed goal and X tracking may activate together");
  require(!f.node->set_parameter(rclcpp::Parameter("arm_tracking_goal_enabled", false)).successful,
    "pending activation must protect the fixed goal mode");
  f.tick();
  require(!f.node->set_parameters_atomically({
      rclcpp::Parameter("arm_tracking_goal_enabled", false),
      rclcpp::Parameter("arm_tracking_enabled", true)}).successful,
    "repeating enable cannot change the active fixed goal mode");
  require(!f.node->set_parameter(rclcpp::Parameter("arm_tracking_x_enabled", false)).successful,
    "an active fixed goal cannot drop its X tracking prerequisite");
  require(f.node->set_parameters_atomically({
      rclcpp::Parameter("arm_tracking_enabled", false),
      rclcpp::Parameter("arm_tracking_x_enabled", false),
      rclcpp::Parameter("arm_tracking_goal_enabled", false)}).successful,
    "an atomic disable may clear both tracking modes");
  f.tick();
  f.enable();
  f.tick();
  f.goal(f.initial.x + 0.05, f.initial.z);
  f.tick(100ms);
  require(!f.controller->position_command_pending(),
    "the dedicated goal topic must not affect a legacy tracking session");
}

void goal_submission_waits_for_settled_activation_feedback()
{
  Fixture f(true);
  f.motors[0]->put<std::int8_t>(0x2707, 0);
  f.motors[0]->put<std::int32_t>(0x606C, 6000);
  f.enable_goal();
  f.goal(f.initial.x + 0.05, f.initial.z);
  require(f.controller->planar_halt_pending() && !f.controller->position_command_pending(),
    "a goal arriving before standstill and baseline acquisition must not queue motion");
  f.motors[0]->put<std::int8_t>(0x2707, 1);
  f.motors[0]->put<std::int32_t>(0x606C, 0);
  f.tick();
  f.goal(f.initial.x + 0.05, f.initial.z);
  f.tick(100ms);
  require(f.controller->position_command_pending(),
    "a fresh goal retry may proceed after settled activation feedback exists");
}

void goal_alignment_holds_arm_and_committed_goal_ignores_live_packets()
{
  Fixture f(true);
  f.enable_goal();
  f.tick();
  auto queued_live = f.message(f.initial.z + 0.05, 0.02);
  queued_live.point.x = f.initial.x + 0.05;
  f.tracker->observe_target(queued_live, f.now);
  f.tick(100ms);
  require(!f.controller->position_command_pending() && f.halted() &&
    f.tracker->lateral_velocity(f.now) > 0.0,
    "before goal commitment, live vision may align Y but must hold arm X/Z");
  const double goal_x = f.initial.x + 0.08, goal_z = f.initial.z + 0.03;
  f.goal(goal_x, goal_z);
  require(f.tracker->lateral_velocity(f.now) == 0.0,
    "committing a fixed goal must immediately clear chassis lateral motion");
  f.tick();
  require(f.controller->position_command_pending(), "the committed goal must queue its complete endpoint");
  f.tracker->observe_target(queued_live, f.now);
  f.target_xz(f.initial.x - 0.1, f.initial.z - 0.1, -0.02);
  f.goal(goal_x, goal_z);
  f.finish_handshake();
  const auto next = f.commanded_pose();
  require(next.x > f.initial.x && next.z > f.initial.z &&
    f.tracker->lateral_velocity(f.now) == 0.0,
    "queued and fresh live packets must not cancel, redirect or restore lateral motion after a goal");
}

void fixed_goal_heartbeats_keep_one_command_beyond_ten_seconds()
{
  Fixture f(true);
  f.enable_goal();
  f.tick();
  const double goal_x = f.initial.x + 0.30, goal_z = f.initial.z;
  const auto triggers = f.motors[0]->triggers;
  f.goal(goal_x, goal_z);
  for (unsigned i = 0; i < 600; ++i) {
    f.tick(20ms, false);
    // The camera may be occluded; only the heartbeat changes, never the destination.
    f.goal(goal_x, goal_z);
    f.tracker->after_update();
    require(f.tracker->lateral_velocity(f.now) == 0.0,
      "fixed goal execution must never command chassis lateral motion");
  }
  require(f.motors[0]->triggers == triggers + 1 && !f.halted() &&
    !f.controller->position_command_pending(),
    "fresh goal heartbeats must preserve one full PP command without a motion duration limit");
  require(std::abs(f.commanded_pose().x - goal_x) < 0.000002 &&
    std::abs(f.controller->actual_pose().x - f.initial.x) < 0.000002,
    "unchanged feedback must neither shrink the endpoint nor cause repeated triggers");
  require(f.controller->fault_reason().empty() && !f.power_off,
    "long-running fresh goal commands must not synthesize a hardware fault");
}

void fixed_goals_submit_the_full_endpoint_once()
{
  for (int z_direction : {-1, 0, 1}) {
    for (const auto elapsed : {20ms, 100ms, 240ms}) {
      Fixture f(true);
      f.measure({0.35, 0.35, 0.0});
      f.enable_goal();
      f.tick();
      const auto baseline = f.controller->actual_pose();
      const double goal_x = baseline.x + 0.03;
      const double goal_z = baseline.z + z_direction * 0.04;
      f.goal(goal_x, goal_z);
      f.tick(elapsed);
      require(f.controller->position_command_pending(), "fixed goal must queue the full endpoint");
      f.goal(goal_x, goal_z);
      f.finish_handshake();
      const auto target = f.commanded_pose();
      require(std::abs(target.x - goal_x) < 0.000002 &&
        std::abs(target.z - goal_z) < 0.000002 &&
        std::abs(target.pitch - baseline.pitch) < 0.000002,
        "fixed-goal submission must preserve the full endpoint and activation pitch");
      const auto triggers = f.motors[0]->triggers;
      for (unsigned i = 0; i < 3; ++i) {
        f.tick(100ms, false);
        f.goal(goal_x, goal_z);
        f.tracker->after_update();
        require(!f.controller->position_command_pending() && f.motors[0]->triggers == triggers,
          "same-goal heartbeats must not retrigger an accepted PP endpoint");
      }
    }
  }
}

void invalid_full_goal_is_rejected_before_motion_and_latches()
{
  for (unsigned invalid = 0; invalid < 4; ++invalid) {
    Fixture f(true);
    f.enable_goal();
    f.tick();
    auto point = f.goal_message(f.initial.x + 0.05, f.initial.z);
    if (invalid == 0) {point.point.x = 10.0;}
    if (invalid == 1) {point.point.z = f.initial.z + 0.16;}
    if (invalid == 2) {point.point.y = 0.01;}
    if (invalid == 3) {point.header.frame_id = "camera";}
    const auto triggers = f.motors[0]->triggers;
    f.tracker->observe_goal(point, f.now);
    f.tick();
    f.goal(f.initial.x + 0.05, f.initial.z);
    f.tick(100ms);
    require(f.halted() && !f.controller->position_command_pending() &&
      f.motors[0]->triggers == triggers && f.tracker->lateral_velocity(f.now) == 0.0,
      "an invalid full goal must latch rejection before any reduced step or later command can move");
    require(f.controller->fault_reason().empty() && !f.power_off,
      "goal input rejection must not be reported as a hardware fault");
  }
}

void endpoint_completion_waits_for_locked_x_and_pitch()
{
  for (bool fixed_goal : {false, true}) {
    for (bool pitch_lags : {false, true}) {
      Fixture f(true);
      f.measure({0.35, 0.35, 0.0});
      if (fixed_goal) {f.enable_goal();} else {f.enable();}
      f.tick();
      const auto baseline = f.controller->actual_pose();
      const double x = baseline.x + (fixed_goal ? 0.03 : 0.0);
      const double z = baseline.z + 0.04;
      if (fixed_goal) {f.goal(x, z);} else {f.target(z);}
      f.tick();
      f.finish_handshake();
      const auto endpoint_joints = f.commanded_joints();
      const auto endpoint = f.commanded_pose();
      const auto triggers = f.motors[0]->triggers;
      if (pitch_lags) {
        auto lagging = endpoint_joints;
        lagging[2] -= 0.1;
        f.measure_joints(lagging);
      } else {
        f.measure({endpoint.x + 0.01, endpoint.z, endpoint.pitch});
      }
      f.tick(20ms, false);
      if (fixed_goal) {f.goal(x, z);} else {f.target(z);}
      f.tracker->after_update();
      require(!f.halted() && !f.controller->planar_halt_pending() &&
        !f.controller->position_command_pending() && f.motors[0]->triggers == triggers,
        "height or X/Z arrival must not halt a full target before locked X and pitch arrive");
      f.measure_joints(endpoint_joints);
      f.tick();
      require(f.halted() && !f.controller->position_command_pending(),
        "the complete endpoint may halt after X, Z and wrapped pitch are within tolerance");
    }
  }
}

void goal_expiry_stop_change_completion_and_replays_require_a_new_session()
{
  for (unsigned terminal = 0; terminal < 6; ++terminal) {
    Fixture f(true);
    f.enable_goal();
    f.tick();
    const double goal_x = f.initial.x + 0.02;
    const auto original = f.goal_message(goal_x, f.initial.z);
    f.tracker->observe_goal(original, f.now);
    f.tick();
    require(f.controller->position_command_pending(), "a valid goal must initially queue motion");
    if (terminal == 0) {
      f.tick(260ms);
    } else if (terminal == 1) {
      f.tracker->stop_tracking(f.now);
    } else if (terminal == 2) {
      f.goal(goal_x + 0.01, f.initial.z);
    } else if (terminal == 3) {
      f.measure({goal_x, f.initial.z, f.initial.pitch});
      f.tick();
    } else if (terminal == 4) {
      f.tracker->observe_goal(original, f.now);
    } else {
      auto stale = f.goal_message(goal_x, f.initial.z);
      stale.header.stamp = f.node->now() - rclcpp::Duration::from_seconds(0.3);
      f.tracker->observe_goal(stale, f.now);
    }
    const auto triggers = f.motors[0]->triggers;
    for (unsigned i = 0; i < 3; ++i) {
      f.tick();
      f.goal(goal_x, f.initial.z);
      f.target_xz(goal_x, f.initial.z, 0.02);
    }
    require(f.halted() && !f.controller->position_command_pending() &&
      f.motors[0]->triggers == triggers && f.tracker->lateral_velocity(f.now) == 0.0,
      "terminal fixed-goal states must reject fresh-stamped cached goals and live images");
    require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", false)).successful,
      "operator disable must end the completed or cancelled goal session");
    f.tick();
    f.enable_goal();
    f.tick();
    const auto baseline = f.controller->actual_pose();
    f.goal(baseline.x + 0.01, baseline.z);
    f.tick(100ms);
    require(f.controller->position_command_pending(),
      "a new enable session may accept a different fixed goal");
  }
}

void live_targets_keep_activation_x_and_pitch_without_arrival_waits()
{
  for (int direction : {-1, 1}) {
    for (bool long_idle : {false, true}) {
      Fixture f(true);
      f.enable();
      f.tick(long_idle ? 1000ms : 20ms);
      f.target(f.initial.z + direction * 0.05);
      f.tick();
      require(f.controller->position_command_pending(), "fresh target must queue in full");
      f.finish_handshake();
      const auto first = f.commanded_pose();
      require(std::abs(first.x - f.initial.x) < 0.000002 &&
        std::abs(first.pitch - f.initial.pitch) < 0.000002 &&
        std::abs(first.z - f.initial.z - direction * 0.05) < 0.000002,
        "Z-only tracking must submit full height and retain activation X/pitch after any idle");
      f.target(f.initial.z + direction * 0.1);
      f.tick();
      require(f.controller->position_command_pending(),
        "a different same-direction target must not wait for measured arrival");
      f.finish_handshake();
      require(std::abs(f.commanded_pose().z - f.initial.z - direction * 0.1) < 0.000002 &&
        std::abs(f.controller->actual_pose().z - f.initial.z) < 0.000002,
        "the full replacement height must reach the drive despite unchanged actual feedback");
      require(f.motors[0]->get<std::uint32_t>(0x6081) == 36000 &&
        f.motors[0]->get<std::uint32_t>(0x6083) == 60000 &&
        f.motors[0]->get<std::uint32_t>(0x6084) == 60000,
        "visual targets must retain the configured motor PP speed and acceleration");
    }
  }
}

void deadband_and_activation_standstill_still_guard_targets()
{
  for (bool waiting_for_halt : {false, true}) {
    Fixture f(true);
    if (waiting_for_halt) {
      f.motors[0]->put<std::int8_t>(0x2707, 0);
      f.motors[0]->put<std::int32_t>(0x606C, 6000);
    }
    f.enable();
    for (unsigned i = 0; i < 10; ++i) {
      f.tick(100ms);
      f.target(f.initial.z + (waiting_for_halt ? 0.05 : 0.001));
      f.tracker->after_update();
      require(!f.controller->position_command_pending(),
        "deadband and pending halt must not queue a target");
    }
    if (waiting_for_halt) {
      f.motors[0]->put<std::int8_t>(0x2707, 1);
      f.motors[0]->put<std::int32_t>(0x606C, 0);
      f.tick(20ms, false);
      require(!f.controller->planar_halt_pending(), "feedback must confirm standstill");
    }
    f.tick(20ms, false);
    f.target(f.initial.z + 0.05);
    f.tracker->after_update();
    require(f.controller->position_command_pending(), "fresh input must permit the full target");
    f.finish_handshake();
    require(std::abs(f.commanded_pose().z - f.initial.z - 0.05) < 0.000002,
      "a complete target must be submitted after the activation and deadband guards clear");
  }
}

void valid_large_joint_changes_are_not_reduced()
{
  Fixture f(true);
  f.measure_joints({0.5 - 5.0 * arm::kPi / 6.0, -0.02 + 5.0 * arm::kPi / 6.0, -0.48});
  f.enable();
  f.tick();
  const auto actual_joints = f.controller->actual_joints();
  const auto baseline = f.controller->actual_pose();
  auto endpoint = baseline;
  endpoint.z -= 0.005;
  const auto full_joints = f.controller->validate_target_pose(endpoint);
  require(std::abs(full_joints[1] - actual_joints[1]) > 3.0 * arm::kPi / 180.0,
    "fixture must require a legal joint change larger than the removed one-degree bound");
  f.target(endpoint.z);
  f.tick();
  require(f.controller->position_command_pending(), "a legal full IK endpoint must proceed");
  f.finish_handshake();
  const auto commanded = f.commanded_joints();
  for (std::size_t i = 0; i < commanded.size(); ++i) {
    require(std::abs(commanded[i] - full_joints[i]) < 0.000002,
      "the drive must receive the complete validated IK solution without halving");
  }
  const auto target = f.commanded_pose();
  require(std::abs(target.z - endpoint.z) < 0.000002 &&
    std::abs(target.x - baseline.x) < 0.000002 &&
    std::abs(target.pitch - baseline.pitch) < 0.000002,
    "large joint changes must still preserve the requested height and locked X/pitch");
}

void valid_activation_pose_corrections_are_not_step_limited()
{
  for (bool pitch_drift : {false, true}) {
    Fixture f(true);
    f.enable();
    auto drifted = f.initial;
    if (pitch_drift) {drifted.pitch += 0.05;} else {drifted.x += 0.02;}
    f.measure(drifted);
    f.tick(20ms, false);
    f.target(f.initial.z + 0.05);
    f.tick();
    require(f.controller->position_command_pending(),
      "a legal correction of locked X or pitch must not hit an artificial joint-step limit");
    f.finish_handshake();
    const auto target = f.commanded_pose();
    require(std::abs(target.x - f.initial.x) < 0.000002 &&
      std::abs(target.z - f.initial.z - 0.05) < 0.000002 &&
      std::abs(target.pitch - f.initial.pitch) < 0.000002,
      "the full target must restore activation X/pitch while moving to the requested height");
    require(f.controller->fault_reason().empty() && !f.power_off,
      "a valid complete correction must not latch tracking or hardware faults");
  }
}

void expiry_precedes_handshake_and_new_capture_is_required()
{
  Fixture f;
  f.start_target();
  require(f.tracker->lateral_velocity(f.now) > 0, "fixture must also command lateral motion");
  f.tick(20ms, false);
  const auto triggers = f.motors[0]->triggers;
  const auto old = f.message(f.initial.z + 0.05);
  f.tick(400ms);
  require(f.halted() && !f.controller->position_command_pending(), "timeout must cancel staging");
  require(f.tracker->lateral_velocity(f.now) == 0.0, "target timeout must stop lateral motion");
  require(f.motors[0]->triggers == triggers, "expired target must never trigger");
  f.tracker->observe_target(old, f.now);
  f.tick();
  require(!f.controller->position_command_pending(), "pre-stop capture must not restart motion");
  f.target(f.initial.z + 0.05);
  f.tick(100ms);
  require(f.controller->position_command_pending(), "new capture may resume after standstill");
}

void malformed_stale_and_replayed_frames_halt()
{
  for (unsigned kind = 0; kind < 9; ++kind) {
    Fixture f;
    f.start_target();
    require(f.tracker->lateral_velocity(f.now) > 0, "fixture must also command lateral motion");
    auto message = f.message(f.initial.z + 0.05);
    switch (kind) {
      case 0: message.header.frame_id = "camera"; break;
      case 1: message.point.z = std::numeric_limits<double>::quiet_NaN(); break;
      case 2: message.point.x = std::numeric_limits<double>::infinity(); break;
      case 3: message.header.stamp = builtin_interfaces::msg::Time{}; break;
      case 4: message.header.stamp = f.node->now() - rclcpp::Duration(1, 0); break;
      case 5: message.header.stamp = f.node->now() + rclcpp::Duration(1, 0); break;
      case 6:
        f.tracker->observe_target(message, f.now);
        break;  // Deliver exactly the same stamp again below.
      case 7: message.point.z = f.initial.z + 0.151; break;
      case 8: message.point.y = std::numeric_limits<double>::quiet_NaN(); break;
    }
    f.tracker->observe_target(message, f.now);
    require(f.halted() && !f.controller->position_command_pending(),
      "invalid target must cancel pending motion immediately");
    require(f.tracker->lateral_velocity(f.now) == 0.0,
      "invalid target must immediately clear its lateral command as well");
    require(f.controller->fault_reason().empty() && !f.power_off,
      "invalid input must not latch a hardware fault");
  }
}

void delayed_captures_cross_the_activation_barrier()
{
  Fixture f(true);
  f.enable();
  f.target(f.initial.z + 0.05);  // Exactly at enable time is still too early.
  require(!f.controller->position_command_pending(), "enable-time capture cannot start motion");
  for (unsigned frame = 1; frame <= 3; ++frame) {
    f.tick(30ms, false);
    auto message = f.message(f.initial.z + 0.05);
    message.header.stamp = f.node->now() - rclcpp::Duration::from_seconds(0.08);
    f.tracker->observe_target(message, f.now);
    f.tracker->after_update();
    require(f.controller->position_command_pending() == (frame == 3),
      "80 ms capture latency at 30 ms cadence must recover on the first post-enable capture");
  }
  f.finish_handshake();
  const auto target = f.commanded_pose();
  require(std::abs(target.z - f.initial.z - 0.05) < 0.000002 &&
    std::abs(target.x - f.initial.x) < 0.000002 &&
    std::abs(target.pitch - f.initial.pitch) < 0.000002,
    "delayed capture recovery must submit full height and preserve locked X/pitch");
}

void delayed_captures_recover_after_a_hold_without_replaying_old_input()
{
  // Exercise explicit stop, direction reversal, and an invalid frame during motion.
  for (unsigned reason = 0; reason < 3; ++reason) {
    Fixture f(true);
    f.enable();
    f.tick(1ms, false);
    const auto accepted = f.message(f.initial.z + 0.05);
    f.tracker->observe_target(accepted, f.now);
    f.tick(100ms);
    require(f.controller->position_command_pending(), "fixture must have a staged motion");
    const double next_z = f.initial.z + (reason == 1 ? -0.05 : 0.05);
    auto queued_before_hold = f.message(next_z);
    if (reason == 0) {
      f.tracker->stop_tracking(f.now);
    } else {
      auto trigger = queued_before_hold;
      if (reason == 2) {trigger.point.z = std::numeric_limits<double>::quiet_NaN();}
      f.tracker->observe_target(trigger, f.now);
    }
    require(f.halted() && !f.controller->position_command_pending(),
      "each hold must immediately cancel the staged motion");

    for (unsigned frame = 1; frame <= 3; ++frame) {
      f.tick(30ms, false);
      auto stale = f.message(next_z);
      stale.header.stamp = f.node->now() - rclcpp::Duration(1, 0);
      auto future = f.message(next_z);
      future.header.stamp = f.node->now() + rclcpp::Duration(1, 0);
      for (const auto & rejected : {queued_before_hold, accepted, stale, future}) {
        f.tracker->observe_target(rejected, f.now);
        f.tracker->after_update();
        require(f.halted() && !f.controller->position_command_pending(),
          "queued, duplicate, stale and future captures must never release a hold");
      }
      auto delayed = f.message(next_z);
      delayed.header.stamp = f.node->now() - rclcpp::Duration::from_seconds(0.08);
      f.tracker->observe_target(delayed, f.now);
      f.tracker->after_update();
      require(f.controller->position_command_pending() == (frame == 3),
        "the first post-hold capture may queue immediately once the standstill guard clears");
    }
    f.tick(20ms);
    require(f.controller->position_command_pending(),
      "the first post-hold capture must recover without moving the capture barrier");
    require(f.controller->fault_reason().empty() && !f.power_off,
      "invalid input and safe recovery must not cause a hardware fault or power cut");
  }
}

void explicit_stop_while_already_held_requires_a_new_capture()
{
  Fixture f(true);
  f.enable();
  f.tick(60ms, false);
  auto before_stop = f.message(f.initial.z + 0.05);
  before_stop.header.stamp = f.node->now() - rclcpp::Duration::from_seconds(0.02);
  f.tracker->stop_tracking(f.now);
  f.tracker->observe_target(before_stop, f.now);
  f.tick(20ms);
  require(f.halted() && !f.controller->position_command_pending(),
    "an explicit stop while already held must invalidate captures newer than enable but older than stop");
  f.tick(60ms, false);
  auto after_stop = f.message(f.initial.z + 0.05);
  after_stop.header.stamp = f.node->now() - rclcpp::Duration::from_seconds(0.04);
  f.tracker->observe_target(after_stop, f.now);
  f.tracker->after_update();
  require(f.controller->position_command_pending(),
    "a valid delayed capture taken after the explicit stop may resume motion");
}

void fresh_live_targets_keep_one_command_beyond_two_seconds()
{
  Fixture f(true);
  f.enable();
  f.tick();
  f.target(f.initial.z + 0.05);
  f.tick();
  f.finish_handshake();
  const auto triggers = f.motors[0]->triggers;
  f.target(f.initial.z + 0.05);
  for (unsigned i = 0; i < 30; ++i) {
    f.tick(100ms, false);
    f.target(f.initial.z + 0.05);
    f.tracker->after_update();
  }
  require(!f.halted() && !f.controller->position_command_pending() &&
    f.motors[0]->triggers == triggers && f.controller->fault_reason().empty() && !f.power_off,
    "fresh same-target captures must preserve one PP command beyond the removed two-second limit");
  require(std::abs(f.commanded_pose().z - f.initial.z - 0.05) < 0.000002,
    "unchanged actual feedback must not replace the full endpoint with partial moves");
  f.tick(260ms);
  require(f.halted() && !f.controller->position_command_pending(),
    "removing motion-duration limits must preserve stale-input shutdown");
}

void reversal_cancels_the_old_direction_before_restarting()
{
  for (bool accepted : {false, true}) {
    Fixture f;
    f.start_target();
    if (accepted) {f.finish_handshake();}
    f.target(f.initial.z - 0.05);
    require(f.halted() && !f.controller->position_command_pending(),
      "opposite target must cancel the old endpoint immediately, even after acceptance");
    f.tick();
    f.target(f.initial.z - 0.05);
    f.tick(100ms);
    f.finish_handshake();
    const auto next = f.commanded_pose();
    require(std::abs(next.z - f.initial.z + 0.05) < 0.000002,
      "the full reverse height target may proceed only after stopping");
  }
}

void activation_waits_for_actual_stop_and_manual_restore_is_fresh()
{
  Fixture f;
  const auto stale_manual = std::vector<double>{f.initial.x, f.initial.z + 0.02, f.initial.pitch};
  require(f.node->set_parameter(rclcpp::Parameter("yiyou_arm_target", stale_manual)).successful,
    "manual target accepted before activation");
  f.tick();
  f.motors[0]->put<std::int8_t>(0x2707, 0);
  f.motors[0]->put<std::int32_t>(0x606C, 6000);
  f.enable();
  f.target(f.initial.z + 0.04);
  f.tick(100ms);
  require(f.controller->planar_halt_pending() && !f.controller->position_command_pending(),
    "activation cannot move before all axes settle");
  require(!f.node->set_parameter(rclcpp::Parameter("yiyou_arm_target", stale_manual)).successful,
    "manual writes must be rejected during visual ownership");
  const arm::Pose settled{f.initial.x + 0.01, f.initial.z + 0.01, 0.03};
  f.measure(settled);
  f.motors[0]->put<std::int8_t>(0x2707, 1);
  f.motors[0]->put<std::int32_t>(0x606C, 0);
  f.tick();
  const auto baseline = f.controller->actual_pose();
  f.target(baseline.z + 0.04);
  f.tick(100ms);
  f.finish_handshake();
  const auto commanded = f.commanded_pose();
  require(std::abs(commanded.x - baseline.x) < 0.000002 &&
    std::abs(commanded.pitch - baseline.pitch) < 0.000002,
    "activation must lock post-stop feedback, not pre-stop coordinates");
  const auto mixed = f.node->set_parameters_atomically({
      rclcpp::Parameter("arm_tracking_enabled", false),
      rclcpp::Parameter("yiyou_arm_target", stale_manual)});
  require(!mixed.successful, "mode changes cannot smuggle manual targets through arbitration");
  require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", false)).successful,
    "disable must commit");
  require(!f.node->set_parameter(rclcpp::Parameter("yiyou_arm_target", stale_manual)).successful,
    "manual writes must wait for ownership restoration");
  f.tick();
  const auto restored = f.node->get_parameter("yiyou_arm_target").as_double_array();
  const auto actual = f.controller->actual_pose();
  require(restored == std::vector<double>({actual.x, actual.z, actual.pitch}),
    "manual parameter baseline must synchronize to measured stopped pose");
  require(!f.controller->position_command_pending() && f.halted(),
    "restoring manual state must not replay an old target");
  auto next = restored;
  next[1] += 0.005;
  require(f.node->set_parameter(rclcpp::Parameter("yiyou_arm_target", next)).successful,
    "manual input must work again after restoration");
  f.tick();
  require(f.controller->position_command_pending(), "fresh manual target resumes normal control");
}

void planar_halt_cancels_each_phase_without_touching_fourth_axis()
{
  for (unsigned ticks = 0; ticks <= 7; ++ticks) {
    Fixture f;
    auto pose = f.initial;
    pose.z += 0.005;
    f.controller->set_target_pose(pose);
    for (unsigned i = 0; i < ticks; ++i) {
      f.now += 20ms;
      f.controller->update(f.now);
    }
    f.controller->set_fourth_axis_jog(1);
    const auto fourth_control = f.motors[3]->get<std::uint16_t>(0x6040);
    const auto fourth_triggers = f.motors[3]->triggers;
    const auto planar_triggers = f.motors[0]->triggers;
    f.controller->halt_planar_position();
    require(f.halted() && !f.controller->position_command_pending(),
      "Halt must cancel queued, staged, triggered and accepted targets");
    require(f.controller->fourth_axis_jog() == 1 &&
      f.motors[3]->get<std::uint16_t>(0x6040) == fourth_control &&
      f.motors[3]->triggers == fourth_triggers,
      "planar Halt must leave fourth-axis state and commands untouched");
    f.now += 20ms;
    f.controller->update(f.now);
    require(!f.controller->planar_halt_pending() &&
      f.motors[0]->triggers == planar_triggers && f.halted(),
      "post-halt feedback must settle without replaying a target");
  }
}

void planar_halt_refreshes_the_ik_seed()
{
  Fixture f;
  auto old_target = f.initial;
  old_target.pitch = -3.0;
  f.controller->set_target_pose(old_target);
  for (unsigned i = 0; i < 7; ++i) {
    f.now += 20ms;
    f.controller->update(f.now);
  }
  require(!f.controller->position_command_pending(), "old target must be accepted");
  auto measured = f.initial;
  measured.pitch = 3.0;
  f.measure(measured);
  f.now += 20ms;
  f.controller->update(f.now);
  bool rejected_old_seed = false;
  try {(void)f.controller->validate_target_pose(measured);} catch (const std::invalid_argument &) {
    rejected_old_seed = true;
  }
  require(rejected_old_seed, "test must distinguish a stale accepted seed from actual feedback");
  f.controller->halt_planar_position();
  f.now += 20ms;
  f.controller->update(f.now);
  (void)f.controller->validate_target_pose(measured);
}

void stop_topic_and_hardware_failure()
{
  Fixture f;
  f.start_target();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(f.node);
  auto publisher = f.node->create_publisher<std_msgs::msg::Empty>("/arm_tracking_stop", 1);
  publisher->publish(std_msgs::msg::Empty{});
  executor.spin_some();
  executor.spin_some();
  require(f.halted() && !f.controller->position_command_pending(), "stop topic must halt now");
  executor.remove_node(f.node);

  Fixture broken;
  broken.motors[0]->fail_halt = true;
  bool threw = false;
  try {broken.controller->halt_planar_position();} catch (const std::exception &) {threw = true;}
  require(threw && broken.power_off && !broken.controller->fault_reason().empty(),
    "Halt I/O failure must retain shared power-off and fault latch");
  broken.motors[0]->fail_halt = false;
}

void unreachable_target_is_input_rejection()
{
  Fixture f;
  f.measure({0.14, std::sqrt(0.7 * 0.7 - 0.14 * 0.14), 0.0});
  f.enable();
  const auto baseline = f.controller->actual_pose();
  f.target(baseline.z + 0.05);
  f.tick(100ms);
  require(f.halted() && !f.controller->position_command_pending() &&
    f.controller->fault_reason().empty() && !f.power_off,
    "unreachable complete target must hold without hardware fault");
  require(f.tracker->lateral_velocity(f.now) == 0.0,
    "unreachable height must not leave chassis lateral motion active");
}

void unreachable_height_is_not_replaced_with_a_partial_move()
{
  Fixture f(true);
  f.measure({0.14, 0.6858, 0.0});
  f.enable();
  f.tick();
  const auto baseline = f.controller->actual_pose();
  const auto triggers = f.motors[0]->triggers;
  f.target(baseline.z + 0.05);
  f.tick();
  require(f.halted() && !f.controller->position_command_pending() &&
    f.motors[0]->triggers == triggers && f.controller->fault_reason().empty(),
    "an unreachable full height must be rejected even when a shorter move would be reachable");
}

void app_stop_discards_unapplied_and_in_flight_targets()
{
  for (const bool fourth_axis : {false, true}) {
    // Zero ticks leaves the parameter committed but unapplied. Later ticks
    // cover queued, staged, triggered and acknowledged PP commands.
    for (unsigned ticks = 0; ticks <= 8; ++ticks) {
      Fixture f(false, fourth_axis);
      chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
        [](Clock::time_point) {return true;});
      app.command(103, f.now);
      const auto requested = f.node->get_parameter("yiyou_arm_target").as_double_array();
      require(requested == std::vector<double>({f.initial.x, f.initial.z + 0.01, f.initial.pitch}),
        "App step must commit the same metre/radian target used by manual control");
      for (unsigned i = 0; i < ticks; ++i) {f.tick();}
      const std::array<unsigned, 3> triggers{
        f.motors[0]->triggers, f.motors[1]->triggers, f.motors[2]->triggers};
      f.motors[0]->put<std::int8_t>(0x2707, 0);
      f.motors[0]->put<std::int32_t>(0x606C, 6000);
      app.command(109, f.now);
      require(f.halted() && !f.controller->position_command_pending(),
        "App stop must cancel a committed or in-flight target on three or four axes");
      f.tick();
      require(f.controller->planar_halt_pending(), "App stop must wait for measured standstill");
      app.command(103, f.now);
      require(f.node->get_parameter("yiyou_arm_target").as_double_array() == requested,
        "App steps during halt must not replace the committed request");
      auto settled = f.initial;
      settled.z += 0.002;
      f.measure(settled);
      f.motors[0]->put<std::int8_t>(0x2707, 1);
      f.motors[0]->put<std::int32_t>(0x606C, 0);
      for (unsigned i = 0; i < 12; ++i) {f.tick();}
      const auto actual = f.controller->actual_pose();
      require(f.node->get_parameter("yiyou_arm_target").as_double_array() ==
        std::vector<double>({actual.x, actual.z, actual.pitch}),
        "App stop must consume the old request by synchronizing the stopped pose");
      require(f.halted() && !f.controller->planar_halt_pending() &&
        !f.controller->position_command_pending(), "App stop must remain held after standstill");
      for (std::size_t i = 0; i < triggers.size(); ++i) {
        require(f.motors[i]->triggers == triggers[i], "App stop must never replay the old target");
      }
      require(!f.power_off && f.controller->fault_reason().empty(),
        "ordinary App stop must hold safely without a hardware fault");
    }
  }
}

void app_fourth_pulses_expire_without_renewal_or_replay()
{
  for (const int command : {107, 108}) {
    for (unsigned ticks = 0; ticks <= 4; ++ticks) {
      Fixture f;
      f.motors[3]->put<std::int32_t>(0x6064, static_cast<std::int32_t>(kPulses));
      chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
        [](Clock::time_point) {return true;});
      const auto start = f.now;
      const auto initial_triggers = f.motors[3]->triggers;
      app.command(command, start);
      const int direction = command == 107 ? 1 : -1;
      require(f.controller->fourth_axis_jog() == direction, "App fourth-axis pulse must start");
      for (unsigned i = 0; i < ticks; ++i) {
        app.before_update(f.now + 20ms);
        f.tick();
      }
      if (ticks == 2) {
        require(f.motors[3]->triggers == initial_triggers,
          "expiry fixture must cover a staged target before its trigger");
      } else if (ticks == 3) {
        require(f.motors[3]->triggers > initial_triggers,
          "expiry fixture must also cover an already triggered target");
      }
      app.command(command, start + 90ms);
      app.command(command == 107 ? 108 : 107, start + 99ms);
      app.before_update(start + 99ms);
      require(f.controller->fourth_axis_jog() == direction,
        "repeated or opposite clicks must not replace the active pulse");
      const auto triggers = f.motors[3]->triggers;
      app.before_update(start + 100ms);
      require(f.controller->fourth_axis_jog() == 0 &&
        (f.motors[3]->get<std::uint16_t>(0x6040) & 0x0100U),
        "repeated clicks must not extend the original 100 ms deadline");
      f.tick(std::chrono::duration_cast<std::chrono::milliseconds>(start + 100ms - f.now));
      for (unsigned i = 0; i < 8; ++i) {
        app.before_update(f.now + 20ms);
        f.tick();
      }
      require(f.motors[3]->triggers == triggers,
        "expired fourth-axis work must never trigger later");
      app.command(command, f.now);
      require(f.controller->fourth_axis_jog() == direction,
        "a genuinely new tap after expiry must be accepted");
      app.stop(f.now);
      require(f.controller->fourth_axis_jog() == 0, "explicit stop must also cancel the new pulse");
    }
  }
}

void app_heartbeat_and_visual_ownership_gate_moves_but_not_stop()
{
  Fixture f;
  bool fresh = false;
  chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
    [&](Clock::time_point) {return fresh;});
  for (int command = 101; command <= 108; ++command) {app.command(command, f.now);}
  require(f.node->get_parameter("yiyou_arm_target").as_double_array().empty() &&
    f.controller->fourth_axis_jog() == 0 && !f.controller->position_command_pending(),
    "without a fresh gateway heartbeat no App movement may be accepted");
  fresh = true;
  app.command(103, f.now);
  f.tick();
  app.command(107, f.now);
  require(f.controller->position_command_pending() && f.controller->fourth_axis_jog() == 1,
    "fresh heartbeat must permit planar and fourth-axis commands");
  fresh = false;
  app.command(109, f.now);
  require(f.halted() && !f.controller->position_command_pending() &&
    f.controller->fourth_axis_jog() == 0, "109 must stop every axis even after heartbeat loss");
  f.tick();
  fresh = true;
  f.enable();
  const auto manual = f.node->get_parameter("yiyou_arm_target").as_double_array();
  app.command(103, f.now);
  require(f.node->get_parameter("yiyou_arm_target").as_double_array() == manual,
    "App planar commands must respect visual ownership");
  app.stop(f.now);
  require(!f.node->get_parameter("arm_tracking_enabled").as_bool() && f.halted(),
    "App stop must revoke visual ownership as well as halt the arm");
}

void app_repeated_stop_is_idle_but_new_targets_are_stopped()
{
  for (const bool fourth_axis : {false, true}) {
    Fixture f(false, fourth_axis);
    chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
      [](Clock::time_point) {return true;});
    const auto io_counts = [&] {
        std::array<std::array<unsigned, 3>, 4> counts{};
        for (std::size_t i = 0; i < f.motors.size(); ++i) {
          counts[i] = {f.motors[i]->reads, f.motors[i]->writes, f.motors[i]->triggers};
        }
        return counts;
      };
    app.command(103, f.now);
    f.tick();
    if (fourth_axis) {app.command(107, f.now);}
    f.motors[0]->put<std::int8_t>(0x2707, 0);
    f.motors[0]->put<std::int32_t>(0x606C, 6000);
    app.command(109, f.now);
    const auto halting_io = io_counts();
    for (unsigned i = 0; i < 20; ++i) {app.command(109, f.now);}
    require(io_counts() == halting_io,
      "repeated App stop while settling must not read, write or trigger hardware");
    f.motors[0]->put<std::int8_t>(0x2707, 1);
    f.motors[0]->put<std::int32_t>(0x606C, 0);
    f.tick();
    const auto stopped_io = io_counts();
    for (unsigned i = 0; i < 20; ++i) {app.command(109, f.now);}
    require(io_counts() == stopped_io,
      "repeated App stop at standstill must not read, write or trigger hardware");

    // A new App target is committed before the periodic parameter application.
    app.command(103, f.now);
    app.command(109, f.now);
    for (unsigned i = 0; i < 10; ++i) {f.tick();}
    require(f.halted() && !f.controller->position_command_pending(),
      "a stop latch must not hide a later App target");

    // External manual requests must also invalidate the idle shortcut, both
    // before application and after the PP acknowledgement has completed.
    for (const bool finish_handshake : {false, true}) {
      const auto actual = f.controller->actual_pose();
      require(f.node->set_parameter(rclcpp::Parameter("yiyou_arm_target",
          std::vector<double>{actual.x, actual.z + 0.01, actual.pitch})).successful,
        "external manual target must be accepted after App stop");
      if (finish_handshake) {
        f.tick();
        f.finish_handshake();
        require(!f.halted(), "external target must have released planar hold");
      }
      const auto before_stop = io_counts();
      app.command(109, f.now);
      for (unsigned i = 0; i < 10; ++i) {f.tick();}
      require(f.halted() && !f.controller->position_command_pending(),
        "repeated App stop must cancel new external manual work");
      const auto stopped = f.controller->actual_pose();
      require(f.node->get_parameter("yiyou_arm_target").as_double_array() ==
        std::vector<double>{stopped.x, stopped.z, stopped.pitch},
        "external target must be consumed rather than replayed after stop");
      for (std::size_t i = 0; i < f.motors.size(); ++i) {
        require(f.motors[i]->triggers == before_stop[i][2],
          "repeated stop must never trigger stale App or external work");
      }
    }
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    defaults_and_startup_guard();
    x_mode_changes_require_disabled_tracking_or_atomic_disable();
    depth_x_moves_both_directions_and_keeps_z_and_pitch();
    depth_diagonal_targets_are_complete_at_any_update_interval();
    latest_depth_target_waits_only_for_the_previous_handshake();
    depth_x_reversal_halts_before_accepting_a_new_capture();
    unreachable_depth_target_is_rejected_without_partial_motion();
    depth_tracking_stop_expiry_and_handover_halt_all_visual_motion();
    goal_mode_requires_x_and_atomic_mode_transitions();
    goal_submission_waits_for_settled_activation_feedback();
    goal_alignment_holds_arm_and_committed_goal_ignores_live_packets();
    fixed_goal_heartbeats_keep_one_command_beyond_ten_seconds();
    fixed_goals_submit_the_full_endpoint_once();
    invalid_full_goal_is_rejected_before_motion_and_latches();
    endpoint_completion_waits_for_locked_x_and_pitch();
    goal_expiry_stop_change_completion_and_replays_require_a_new_session();
    lateral_direction_limits_and_independent_deadband();
    lateral_waits_for_halt_and_exit_requires_neutral();
    manual_motion_and_watchdog_latch_both_visual_axes();
    explicit_stop_clears_lateral_and_requires_a_new_capture();
    lateral_motion_continues_until_disabled_and_recovers_after_loss();
    live_targets_keep_activation_x_and_pitch_without_arrival_waits();
    deadband_and_activation_standstill_still_guard_targets();
    valid_large_joint_changes_are_not_reduced();
    valid_activation_pose_corrections_are_not_step_limited();
    expiry_precedes_handshake_and_new_capture_is_required();
    malformed_stale_and_replayed_frames_halt();
    delayed_captures_cross_the_activation_barrier();
    delayed_captures_recover_after_a_hold_without_replaying_old_input();
    explicit_stop_while_already_held_requires_a_new_capture();
    fresh_live_targets_keep_one_command_beyond_two_seconds();
    reversal_cancels_the_old_direction_before_restarting();
    activation_waits_for_actual_stop_and_manual_restore_is_fresh();
    planar_halt_cancels_each_phase_without_touching_fourth_axis();
    planar_halt_refreshes_the_ik_seed();
    stop_topic_and_hardware_failure();
    unreachable_target_is_input_rejection();
    unreachable_height_is_not_replaced_with_a_partial_move();
    app_stop_discards_unapplied_and_in_flight_targets();
    app_fourth_pulses_expire_without_renewal_or_replay();
    app_heartbeat_and_visual_ownership_gate_moves_but_not_stop();
    app_repeated_stop_is_idle_but_new_targets_are_stopped();
    std::cout << "arm visual tracking tests passed\n";
  } catch (const std::exception & error) {
    std::cerr << "arm visual tracking: " << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
