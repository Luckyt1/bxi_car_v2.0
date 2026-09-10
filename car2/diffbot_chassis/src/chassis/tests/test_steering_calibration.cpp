#include "fake_iswv.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
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

void successful_calibration_runs_in_two_pairs()
{
  chassis::test::FakeCan0 fake;
  const auto midpoints = chassis::test::run_calibration(fake, [] {return true;});

  require(midpoints == std::array<std::int32_t, 4>{0, 0, 0, 0}, "midpoints returned");
  require(
    fake.nonzero_velocity_nodes.size() >= 4 &&
    fake.nonzero_velocity_nodes[0] == 1 && fake.nonzero_velocity_nodes[1] == 2,
    "first calibration pair must be front-left and rear-left");
  require(
    std::find(fake.nonzero_velocity_nodes.begin(), fake.nonzero_velocity_nodes.end(), 3) !=
    fake.nonzero_velocity_nodes.end() &&
    std::find(fake.nonzero_velocity_nodes.begin(), fake.nonzero_velocity_nodes.end(), 4) !=
    fake.nonzero_velocity_nodes.end(),
    "second calibration pair must be rear-right and front-right");
  require(
    fake.max_simultaneous_velocity_nodes == 2,
    "at most two steering axes may receive non-zero calibration velocity together");
  require(
    !fake.second_pair_started_before_first_pair_stopped,
    "the second pair must not move before the first pair has stopped");
  const auto first_stop =
    std::find(fake.zero_velocity_nodes.begin(), fake.zero_velocity_nodes.end(), 1);
  require(first_stop != fake.zero_velocity_nodes.end(), "first axis must stop at its early limit");
  require(
    std::find(first_stop, fake.zero_velocity_nodes.end(), 2) != fake.zero_velocity_nodes.end() &&
    std::find(first_stop, fake.zero_velocity_nodes.end(), 3) != fake.zero_velocity_nodes.end() &&
    std::find(first_stop, fake.zero_velocity_nodes.end(), 4) != fake.zero_velocity_nodes.end(),
    "later axes must continue until their own limits");
  for (std::uint8_t node_id = 1; node_id <= 4; ++node_id) {
    require(
      std::find(fake.position_nodes.begin(), fake.position_nodes.end(), node_id) !=
      fake.position_nodes.end(),
      "all axes get midpoint PDO");
  }
  for (const auto & axis : fake.axes) {
    require(axis.status == 0x0037, "successful core leaves axes operation-enabled");
    require(axis.position_pdo_frames == 2, "each axis receives midpoint prepare/start commands");
  }
}

void protection_read_failure_reports_objects_and_stops()
{
  chassis::test::FakeCan0 fake;
  fake.axes[0].abort_protection_read = true;
  std::string message;
  try {
    (void)chassis::test::run_calibration(fake, [] {return true;});
  } catch (const std::runtime_error & error) {
    message = error.what();
  }
  require(message.find("front_left (Node-ID 1)") != std::string::npos,
    "read failure identifies wheel and node");
  require(message.find("error_status(0x2601):") != std::string::npos &&
    message.find("error_status_2(0x2602):") != std::string::npos,
    "read failure identifies each failed protection object");
  require(message.find("abort") != std::string::npos,
    "read failure preserves underlying SDO error");
  for (const auto & axis : fake.axes) {
    require(axis.zero_velocity_sdo_writes > 0, "read failure commands zero velocity");
    require(axis.quick_stops > 0, "read failure quick-stops all axes");
    require(axis.shutdowns > 0, "read failure disables all axes");
  }
}

void torque_threshold_does_not_wait_for_stall()
{
  chassis::test::FakeCan0 fake;
  auto config = fake.config();
  config.calibration_stall_time_s = 60.0;
  config.calibration_timeout_s = 2.0;

  iswv::canopen::CanopenMaster master(fake.transport);
  iswv::SteeringLayout steering(master, config.motor);
  const auto midpoints = chassis::calibrate_steering(steering, config, [] {return true;});
  require(
    midpoints == std::array<std::int32_t, 4>{0, 0, 0, 0},
    "torque threshold must detect limits without waiting for the legacy stall timer");
}

void calibration_result_returns_measured_limits()
{
  chassis::test::FakeCan0 fake;
  auto config = fake.config();

  iswv::canopen::CanopenMaster master(fake.transport);
  iswv::SteeringLayout steering(master, config.motor);
  const auto result = chassis::calibrate_steering_with_limits(
    steering, config, [] {return true;});

  require(
    result.negative_limits == std::array<std::int32_t, 4>{-100, -200, -300, -400},
    "negative limits returned");
  require(
    result.positive_limits == std::array<std::int32_t, 4>{100, 200, 300, 400},
    "positive limits returned");
  require(
    result.midpoints == std::array<std::int32_t, 4>{0, 0, 0, 0},
    "midpoints returned in full calibration result");
}

void early_second_contact_reports_hard_limit_error()
{
  chassis::test::FakeCan0 fake;
  auto config = fake.config();
  config.calibration_min_limit_separation_rad = 20.0;
  config.calibration_timeout_s = 2.0;

  bool reported = false;
  try {
    iswv::canopen::CanopenMaster master(fake.transport);
    iswv::SteeringLayout steering(master, config.motor);
    (void)chassis::calibrate_steering(steering, config, [] {return true;});
  } catch (const std::runtime_error & error) {
    reported = std::string(error.what()).find("硬限位异常") != std::string::npos;
  }
  require(reported, "an early second contact must report an explicit hard-limit error");
}

void cancellation_stops_all_axes_after_motion_starts()
{
  chassis::test::FakeCan0 fake;
  bool threw = false;
  try {
    (void)chassis::test::run_calibration(
      fake, [&fake] {
        return fake.nonzero_velocity_nodes.empty();
      });
  } catch (const std::runtime_error &) {
    threw = true;
  }
  require(threw, "cancellation must throw");
  for (const auto & axis : fake.axes) {
    require(axis.zero_velocity_sdo_writes > 0, "cancel cleanup writes zero velocity");
    require(axis.quick_stops > 0, "cancel cleanup quick-stops");
    require(axis.shutdowns > 0, "cancel cleanup disables");
  }
}

void midpoint_requires_all_axes_currently_at_target()
{
  chassis::test::FakeCan0 fake;
  fake.axes[0].drift_after_midpoint_reached = true;
  auto config = fake.config();
  config.calibration_timeout_s = 1.0;

  bool threw = false;
  try {
    iswv::canopen::CanopenMaster master(fake.transport);
    iswv::SteeringLayout steering(master, config.motor);
    (void)chassis::calibrate_steering(steering, config, [] {return true;});
  } catch (const std::runtime_error &) {
    threw = true;
  }
  require(threw, "axis drifting away after first arrival must not count as still reached");
  for (const auto & axis : fake.axes) {
    require(axis.quick_stops > 0, "midpoint timeout cleanup quick-stops all axes");
  }
}

void early_cancel_still_stops_constructed_axes()
{
  chassis::test::FakeCan0 fake;
  unsigned calls = 0;
  bool threw = false;
  try {
    (void)chassis::test::run_calibration(
      fake, [&calls] {
        ++calls;
        return calls < 2;
      });
  } catch (const std::runtime_error &) {
    threw = true;
  }
  require(threw, "early cancellation must throw");
  for (const auto & axis : fake.axes) {
    require(axis.zero_velocity_sdo_writes > 0, "early cancel cleanup writes zero velocity");
    require(axis.quick_stops > 0, "early cancel cleanup quick-stops");
    require(axis.shutdowns > 0, "early cancel cleanup disables");
  }
}

}  // namespace

int main()
{
  protection_read_failure_reports_objects_and_stops();
  successful_calibration_runs_in_two_pairs();
  torque_threshold_does_not_wait_for_stall();
  calibration_result_returns_measured_limits();
  early_second_contact_reports_hard_limit_error();
  cancellation_stops_all_axes_after_motion_starts();
  midpoint_requires_all_axes_currently_at_target();
  early_cancel_still_stops_constructed_axes();
  return 0;
}
