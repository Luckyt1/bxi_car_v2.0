#include "arm/kinematics.h"
#include "motor/bxi/motor.h"
#include "motor/yiyou/communication.h"
#include "chassis/steering_target_limits.hpp"
#include "../support/bxi_fake_transport.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

template<typename Function>
void rejects(Function function)
{
  bool rejected = false;
  try {
    function();
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected, "invalid input must throw std::invalid_argument");
}

bool near(double actual, double expected, double tolerance = 1.0e-8)
{
  return std::abs(actual - expected) <= tolerance;
}

void bxi_command_endpoints()
{
  bxi::Command low;
  low.position = -12.5F;
  low.velocity = -45.0F;
  low.torque = -40.0F;
  const auto packed_low = bxi::pack_command(low);
  require(packed_low.has_value(), "minimum MIT command must encode");
  require(packed_low.value() == std::array<std::uint8_t, 8>{}, "minimum encodes as zero");

  bxi::Command high;
  high.position = 12.5F;
  high.velocity = 45.0F;
  high.kp = 500.0F;
  high.kd = 5.0F;
  high.torque = 40.0F;
  const auto packed_high = bxi::pack_command(high);
  require(packed_high.has_value(), "maximum MIT command must encode");
  require(
    packed_high.value() == std::array<std::uint8_t, 8>{
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, "maximum encodes as all ones");

  bxi::Command collision = high;
  collision.ranges.torque = {0.0F, 4095.0F};
  collision.torque = 4092.0F;
  const auto packed_collision = bxi::pack_command(collision);
  require(
    !packed_collision && packed_collision.error().code == bxi::ErrorCode::invalid_argument,
    "ordinary command cannot encode the enter-mode control frame");
}

void bxi_invalid_commands_send_nothing()
{
  bxi_test::FakeTransport transport;
  bxi::Motor motor(transport, 0x123);
  std::array<bxi::Command, 8> invalid{};
  invalid[0].position = 12.6F;
  invalid[1].velocity = -46.0F;
  invalid[2].kp = -1.0F;
  invalid[3].kd = 5.1F;
  invalid[4].torque = 41.0F;
  invalid[5].position = std::numeric_limits<float>::quiet_NaN();
  invalid[6].velocity = std::numeric_limits<float>::infinity();
  invalid[7].ranges.position = {1.0F, 1.0F};
  for (const auto & command : invalid) {
    const auto result = motor.command(command);
    require(
      !result && result.error().code == bxi::ErrorCode::invalid_argument,
      "invalid MIT command must return invalid_argument");
    require(transport.sent_frames().empty(), "invalid command must not reach transport");
  }
  require(motor.command(bxi::Command{}).has_value(), "valid command must send");
  require(transport.sent_frames().size() == 1, "valid command sends exactly one frame");
}

void bxi_control_helpers_send_mit_frames()
{
  bxi_test::FakeTransport transport;
  bxi::Motor motor(transport, 0x123, {false, false});
  require(
    motor.set_velocity(-45.0F, 5.0F, bxi::Model::BXI5014_19).has_value(),
    "velocity control sends");
  require(
    motor.set_position(12.5F, 500.0F, 5.0F, bxi::Model::BXI5014_19).has_value(),
    "position control sends");
  require(
    motor.set_torque(160.0F, bxi::Model::BXI8515_19).has_value(),
    "torque control uses the selected model range");
  const auto frames = transport.sent_frames();
  // Independent wire expectations: unused targets encode zero; inactive gains encode zero.
  const std::array<std::array<std::uint8_t, 8>, 3> expected{{
    {0x7f, 0xff, 0x00, 0x00, 0x00, 0xff, 0xf7, 0xff},
    {0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xf7, 0xff},
    {0x7f, 0xff, 0x7f, 0xf0, 0x00, 0x00, 0x0f, 0xff}}};
  require(frames.size() == expected.size(), "each control call sends exactly one frame");
  for (std::size_t i = 0; i < frames.size(); ++i) {
    require(
      frames[i].id == 0x123 && frames[i].size == 8 &&
      !frames[i].fd && !frames[i].bitrate_switch,
      "control helpers preserve motor ID and CAN options");
    require(
      std::equal(expected[i].begin(), expected[i].end(), frames[i].data.begin()),
      "control helper MIT fields match the requested control type");
  }
}

void bxi_control_helpers_select_model_ranges()
{
  struct ModelLimits
  {
    bxi::Model model;
    float kd_max;
    float torque_max;
  };
  const std::array<ModelLimits, 4> models{{
    {bxi::Model::BXI5014_19, 5.0F, 40.0F},
    {bxi::Model::BXI5018_19, 5.0F, 40.0F},
    {bxi::Model::BXI7010_19, 5.0F, 80.0F},
    {bxi::Model::BXI8515_19, 20.0F, 160.0F}}};
  bxi_test::FakeTransport transport;
  bxi::Motor motor(transport, 1);
  for (const auto & item : models) {
    transport.clear_sent();
    require(
      motor.set_velocity(0.0F, item.kd_max, item.model).has_value(),
      "velocity accepts this model's maximum kd");
    require(
      motor.set_position(0.0F, 1.0F, item.kd_max, item.model).has_value(),
      "position accepts this model's maximum kd");
    require(
      motor.set_torque(item.torque_max, item.model).has_value(),
      "torque accepts this model's maximum torque");
    const auto frames = transport.sent_frames();
    require(frames.size() == 3, "each model sends three control frames");
    for (std::size_t i = 0; i < 2; ++i) {
      require(
        frames[i].data[5] == 0xff && (frames[i].data[6] >> 4) == 0x0f,
        "maximum kd uses the full 12-bit field for this model");
    }
    require(
      (frames[2].data[6] & 0x0f) == 0x0f && frames[2].data[7] == 0xff,
      "maximum torque uses the full 12-bit field for this model");
    require(
      !motor.set_velocity(0.0F, item.kd_max + 1.0F, item.model) &&
      !motor.set_position(0.0F, 1.0F, item.kd_max + 1.0F, item.model) &&
      !motor.set_torque(item.torque_max + 1.0F, item.model),
      "control helpers reject values outside this model's ranges");
    require(transport.sent_frames().size() == 3, "out-of-range values send nothing");
  }
}

void bxi_control_helpers_reject_invalid_and_propagate_errors()
{
  bxi_test::FakeTransport transport;
  bxi::Motor motor(transport, 1);
  const auto model = bxi::Model::BXI5014_19;
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  const std::array<bxi::Result<void>, 6> invalid{
    motor.set_velocity(46.0F, 1.0F, model), motor.set_velocity(1.0F, nan, model),
    motor.set_position(13.0F, 1.0F, 1.0F, model),
    motor.set_position(0.0F, -1.0F, 1.0F, model),
    motor.set_torque(41.0F, model), motor.set_torque(nan, model)};
  for (const auto & result : invalid) {
    require(
      !result && result.error().code == bxi::ErrorCode::invalid_argument,
      "control helpers reject invalid targets and gains");
  }
  const auto unknown = static_cast<bxi::Model>(-1);
  rejects([&] {motor.set_velocity(0.0F, 1.0F, unknown);});
  rejects([&] {motor.set_position(0.0F, 1.0F, 1.0F, unknown);});
  rejects([&] {motor.set_torque(0.0F, unknown);});
  require(transport.sent_frames().empty(), "invalid control calls send nothing");
  transport.close();
  const std::array<bxi::Result<void>, 3> closed{
    motor.set_velocity(0.0F, 1.0F, model), motor.set_position(0.0F, 1.0F, 1.0F, model),
    motor.set_torque(0.0F, model)};
  for (const auto & result : closed) {
    require(
      !result && result.error().code == bxi::ErrorCode::transport_closed,
      "control helpers propagate transport errors");
  }
}

void bxi_explicit_enable_disable_frames()
{
  bxi_test::FakeTransport transport;
  {
    bxi::Motor motor(transport, 0x7ff, {false, false});
    require(transport.sent_frames().empty(), "construction must not enable a motor");
    require(motor.enter_motor_mode().has_value(), "explicit enable must send");
    require(motor.exit_motor_mode().has_value(), "explicit disable must send");
  }
  const auto frames = transport.sent_frames();
  require(frames.size() == 2, "destruction must not send an implicit motor command");
  for (std::size_t i = 0; i < frames.size(); ++i) {
    const auto & frame = frames[i];
    require(
      frame.id == 0x7ff && frame.size == 8 && !frame.fd && !frame.bitrate_switch &&
      !frame.extended && !frame.remote, "special frames preserve selected standard CAN format");
    require(
      std::all_of(
        frame.data.begin(), frame.data.begin() + 7,
        [](std::uint8_t byte) {
          return byte == 0xff;
        }), "special frame prefix must be seven FF bytes");
    require(frame.data[7] == (i == 0 ? 0xfc : 0xfd), "enable/disable opcodes must remain FC/FD");
  }
}

void bxi_feedback_boundaries_and_validation()
{
  bxi::CanFrame frame{};
  frame.id = 0x11;
  frame.size = 5;
  frame.data[0] = 7;
  const auto low = bxi::decode_feedback(frame);
  require(
    low && low.value().prefix == 7 && near(low.value().position, -12.5) &&
    near(low.value().velocity, -45.0), "zero feedback decodes minimum position and velocity");
  std::fill(frame.data.begin() + 1, frame.data.begin() + 5, 0xff);
  const auto high = bxi::decode_feedback(frame);
  require(
    high && near(high.value().position, 12.5) && near(high.value().velocity, 45.0),
    "full-scale feedback decodes maximum position and velocity");

  for (unsigned kind = 0; kind < 5; ++kind) {
    auto invalid = frame;
    if (kind == 0) {invalid.size = 4;}
    if (kind == 1) {invalid.extended = true;}
    if (kind == 2) {invalid.remote = true;}
    if (kind == 3) {invalid.error = true;}
    if (kind == 4) {invalid.id = 0x800;}
    const auto result = bxi::decode_feedback(invalid);
    require(
      !result && result.error().code == bxi::ErrorCode::protocol_error,
      "malformed feedback must return protocol_error");
  }
}

void arm_forward_inverse_round_trip()
{
  arm::Configuration config;
  config.upper_arm = 0.3;
  config.forearm = 0.2;
  config.tool = 0.1;
  config.zero_offsets = {0.0, 0.0, 0.0};
  config.limits = {arm::JointLimit{}, arm::JointLimit{}, arm::JointLimit{}};
  const arm::Kinematics solver(config);
  const auto straight = solver.forward({0.0, 0.0, 0.0});
  require(
    near(straight.x, 0.6) && near(straight.z, 0.0) && near(straight.pitch, 0.0),
    "straight arm must equal the sum of link lengths on +X");
  for (const arm::JointAngles joints : {arm::JointAngles{0.2, 0.6, -0.3},
      arm::JointAngles{0.4, -0.8, 0.2}})
  {
    const auto target = solver.forward(joints);
    const auto inverse = solver.inverse(target, joints);
    require(
      inverse.status == arm::InverseStatus::Success && !inverse.solutions.empty(),
      "reachable pose must have an inverse solution");
    const auto & recovered = inverse.solutions.front().joints;
    for (std::size_t joint = 0; joint < joints.size(); ++joint) {
      require(
        near(
          recovered[joint],
          joints[joint]), "seeded inverse must recover the original branch");
    }
    const auto round_trip = solver.forward(recovered);
    require(
      near(round_trip.x, target.x) && near(round_trip.z, target.z) &&
      near(round_trip.pitch, target.pitch), "inverse solution must reproduce its target pose");
  }
  const auto unreachable = solver.inverse({1.0, 0.0, 0.0});
  require(
    unreachable.status == arm::InverseStatus::Unreachable && unreachable.solutions.empty(),
    "a target beyond total arm length must be rejected");
}

void steering_margin_boundaries()
{
  for (int sign : {-1, 1}) {
    const auto limits = chassis::inset_steering_targets(-1000, 1000, 0, sign, 100.0, 0.101);
    require(
      limits.lower_inc == -989 && limits.upper_inc == 989,
      "nonintegral margin rounds inward to preserve the minimum inset");
    require(
      near(limits.angles.lower, -9.89) && near(limits.angles.upper, 9.89),
      "either encoder sign must preserve ordered angle limits");
  }
  const auto unchanged = chassis::inset_steering_targets(-100, 100, 0, 1, 100.0, 0.0);
  require(
    unchanged.lower_inc == -100 && unchanged.upper_inc == 100,
    "zero margin preserves calibration boundaries");
  rejects([] {chassis::inset_steering_targets(-100, 100, 0, 1, 100.0, 1.0);});
  rejects([] {chassis::inset_steering_targets(-100, 100, 99, 1, 100.0, 0.1);});
  rejects([] {chassis::inset_steering_targets(-100, 100, 0, 0, 100.0, 0.1);});
  rejects([] {chassis::inset_steering_targets(-100, 100, 0, 1, 0.0, 0.1);});
  rejects([] {chassis::inset_steering_targets(-100, 100, 0, 1, 100.0, -0.1);});
}

void ethercat_selection_is_validated_offline()
{
  using chassis::ethercat::Options;
  using chassis::ethercat::validate_selection;
  validate_selection(Options{"enp1s0", {1, 199}, 199});
  validate_selection(Options{"eth0", {1}, std::nullopt});
  for (const auto & invalid : {
      Options{"", {1}, std::nullopt}, Options{"lo", {1}, std::nullopt},
      Options{"invalid interface", {1}, std::nullopt},
      Options{"abcdefghijklmnop", {1}, std::nullopt},
      Options{"eth0", {}, std::nullopt}, Options{"eth0", {0}, std::nullopt},
      Options{"eth0", {200}, std::nullopt}, Options{"eth0", {1, 1}, std::nullopt},
      Options{"eth0", {1}, 2}})
  {
    rejects([&] {validate_selection(invalid);});
  }
}
}  // namespace

int main()
{
  const std::array<std::pair<const char *, void (*)()>, 10> tests{{
    {"BXI command endpoints", bxi_command_endpoints},
    {"BXI invalid commands send nothing", bxi_invalid_commands_send_nothing},
    {"BXI control helper MIT frames", bxi_control_helpers_send_mit_frames},
    {"BXI control helper model ranges", bxi_control_helpers_select_model_ranges},
    {"BXI control helper errors", bxi_control_helpers_reject_invalid_and_propagate_errors},
    {"BXI explicit enable/disable frames", bxi_explicit_enable_disable_frames},
    {"BXI feedback boundaries and validation", bxi_feedback_boundaries_and_validation},
    {"arm FK/IK round trip", arm_forward_inverse_round_trip},
    {"steering margin boundaries", steering_margin_boundaries},
    {"EtherCAT selection validation", ethercat_selection_is_validated_offline}}};
  unsigned failures = 0;
  for (const auto & test : tests) {
    try {
      test.second();
      std::cout << "PASS: " << test.first << '\n';
    } catch (const std::exception & error) {
      ++failures;
      std::cerr << "FAIL: " << test.first << ": " << error.what() << '\n';
    }
  }
  return failures == 0 ? 0 : 1;
}
