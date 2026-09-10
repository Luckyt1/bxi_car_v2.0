#include "chassis/steering_direction.hpp"
#include "fake_iswv.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

template<typename Function>
void rejects(Function function, const char * message)
{
  try {
    function();
  } catch (const std::exception &) {
    return;
  }
  throw std::runtime_error(message);
}

chassis::SteeringConfigFile high_resolution_config(chassis::test::FakeCan0 & fake)
{
  fake.axes[0].negative_limit = -200;
  fake.axes[0].positive_limit = 200;
  auto config = fake.config();
  config.motor.encoder_resolution = 1000;
  config.calibration_position_epsilon_inc = 0;
  return config;
}

chassis::SteeringCalibrationResult calibrate(
  iswv::SteeringLayout & steering, const chassis::SteeringConfigFile & config)
{
  return chassis::calibrate_steering_with_limits(
    steering, config, [] {return true;});
}

std::array<unsigned, 4> position_frame_counts(const chassis::test::FakeCan0 & fake)
{
  return {
    fake.axes[0].position_pdo_frames,
    fake.axes[1].position_pdo_frames,
    fake.axes[2].position_pdo_frames,
    fake.axes[3].position_pdo_frames};
}

void positive_direction_jog_runs_after_fresh_calibration_for_each_wheel()
{
  for (std::size_t index = 0; index < 4; ++index) {
    chassis::test::FakeCan0 fake;
    const auto config = high_resolution_config(fake);
    iswv::canopen::CanopenMaster master(fake.transport);
    iswv::SteeringLayout steering(master, config.motor);
    const auto limits = calibrate(steering, config);
    const auto before = position_frame_counts(fake);

    const double moved = chassis::test_positive_steering_direction(
      steering, config, limits, index, [] {return true;});

    require(moved > 49.5 && moved <= 50.0, "positive jog reports a 50 degree move");
    for (std::size_t i = 0; i < 4; ++i) {
      const auto expected = before[i] + (i == index ? 2U : 0U);
      require(fake.axes[i].position_pdo_frames == expected, "only selected wheel gets jog PDOs");
      if (i == index) {
        require(fake.axes[i].position > limits.midpoints[i], "selected wheel moves positive");
        require(
          fake.axes[i].target_position == fake.axes[i].position,
          "selected wheel reaches target");
      } else {
        require(fake.axes[i].position == limits.midpoints[i], "other wheels stay at midpoint");
      }
    }
  }
}

void one_calibration_supports_four_positive_jogs_in_sequence()
{
  chassis::test::FakeCan0 fake;
  const auto config = high_resolution_config(fake);
  iswv::canopen::CanopenMaster master(fake.transport);
  iswv::SteeringLayout steering(master, config.motor);
  const auto limits = calibrate(steering, config);

  for (std::size_t index = 0; index < 4; ++index) {
    const auto before_frames = position_frame_counts(fake);
    const auto before_positions = std::array<std::int32_t, 4>{
      fake.axes[0].position, fake.axes[1].position, fake.axes[2].position, fake.axes[3].position};
    const auto before_targets = std::array<std::int32_t, 4>{
      fake.axes[0].target_position,
      fake.axes[1].target_position,
      fake.axes[2].target_position,
      fake.axes[3].target_position};

    const double moved = chassis::test_positive_steering_direction(
      steering, config, limits, index, [] {return true;});

    require(moved > 49.5 && moved <= 50.0, "sequential positive jog reports expected motion");
    for (std::size_t i = 0; i < 4; ++i) {
      if (i == index) {
        require(
          fake.axes[i].position_pdo_frames == before_frames[i] + 2U,
          "current wheel receives one jog command pair");
        require(fake.axes[i].position > limits.midpoints[i], "current wheel moves positive");
      } else {
        require(
          fake.axes[i].position_pdo_frames == before_frames[i],
          "other wheels receive no new jog command");
        require(fake.axes[i].position == before_positions[i], "other wheels hold position");
        require(fake.axes[i].target_position == before_targets[i], "other wheels hold target");
      }
    }
  }
}

void cancellation_is_rejected_before_jog_motion()
{
  chassis::test::FakeCan0 fake;
  const auto config = high_resolution_config(fake);
  iswv::canopen::CanopenMaster master(fake.transport);
  iswv::SteeringLayout steering(master, config.motor);
  const auto limits = calibrate(steering, config);
  const auto before = position_frame_counts(fake);

  rejects(
    [&] {
      (void)chassis::test_positive_steering_direction(
        steering, config, limits, 0, [] {return false;});
    },
    "cancelled direction test must throw");

  require(position_frame_counts(fake) == before, "cancel before start must not add jog PDOs");
}

void cancellation_during_jog_stops_and_disables_axis()
{
  chassis::test::FakeCan0 fake;
  const auto config = high_resolution_config(fake);
  iswv::canopen::CanopenMaster master(fake.transport);
  iswv::SteeringLayout steering(master, config.motor);
  const auto limits = calibrate(steering, config);
  const auto before = position_frame_counts(fake);
  unsigned calls = 0;

  rejects(
    [&] {
      (void)chassis::test_positive_steering_direction(
        steering, config, limits, 0,
        [&calls] {
          ++calls;
          return calls <= 3;
        });
    },
    "cancelled direction test must throw after jog command");

  require(
    fake.axes[0].position_pdo_frames == before[0] + 2U,
    "jog command is sent before cancel");
  require(fake.axes[0].quick_stops > 0, "cancel during jog must quick-stop selected axis");
  require(fake.axes[0].shutdowns > 0, "cancel during jog must disable selected axis");
  require(
    iswv::cia402::decode_state(fake.axes[0].status) !=
    iswv::cia402::DriveState::operation_enabled,
    "cancel during jog must leave selected axis not operation-enabled");
}

void stalled_jog_reports_position_and_stops()
{
  chassis::test::FakeCan0 fake;
  const auto config = high_resolution_config(fake);
  iswv::canopen::CanopenMaster master(fake.transport);
  iswv::SteeringLayout steering(master, config.motor);
  const auto limits = calibrate(steering, config);
  auto & axis = fake.axes[0];
  axis.freeze_position = true;
  const auto stops = axis.quick_stops;
  const auto shutdowns = axis.shutdowns;
  std::string message;
  try {
    (void)chassis::test_positive_steering_direction(
      steering, config, limits, 0, [] {return true;});
  } catch (const std::runtime_error & error) {
    message = error.what();
  }
  require(message.find("positive jog timed out: wheel=1 node=1") != std::string::npos,
    "timeout identifies selected axis");
  require(message.find("position_inc=0") != std::string::npos &&
    message.find("moved_deg=0") != std::string::npos &&
    message.find("remaining_deg=") != std::string::npos,
    "timeout reports stationary position and remaining motion");
  require(axis.quick_stops > stops && axis.shutdowns > shutdowns,
    "timeout stops and disables selected axis");
}

void invalid_index_is_rejected_without_motion()
{
  chassis::test::FakeCan0 fake;
  const auto config = high_resolution_config(fake);
  iswv::canopen::CanopenMaster master(fake.transport);
  iswv::SteeringLayout steering(master, config.motor);
  const auto limits = calibrate(steering, config);
  const auto before = position_frame_counts(fake);

  rejects(
    [&] {
      (void)chassis::test_positive_steering_direction(
        steering, config, limits, 4, [] {return true;});
    },
    "invalid wheel index must throw");

  require(position_frame_counts(fake) == before, "invalid index must not add jog PDOs");
}

void insufficient_positive_limit_is_rejected_before_motion()
{
  chassis::test::FakeCan0 fake;
  const auto config = high_resolution_config(fake);
  iswv::canopen::CanopenMaster master(fake.transport);
  iswv::SteeringLayout steering(master, config.motor);
  auto limits = calibrate(steering, config);
  limits.positive_limits[0] = limits.midpoints[0] + 120;
  const auto before = position_frame_counts(fake);

  rejects(
    [&] {
      (void)chassis::test_positive_steering_direction(
        steering, config, limits, 0, [] {return true;});
    },
    "positive jog outside calibrated limit must throw");

  require(position_frame_counts(fake) == before, "limit rejection must happen before jog PDOs");
  require(
    fake.axes[0].position == limits.midpoints[0],
    "limit rejection must leave position centered");
}
}  // namespace

int main()
{
  positive_direction_jog_runs_after_fresh_calibration_for_each_wheel();
  one_calibration_supports_four_positive_jogs_in_sequence();
  cancellation_is_rejected_before_jog_motion();
  cancellation_during_jog_stops_and_disables_axis();
  stalled_jog_reports_position_and_stops();
  invalid_index_is_rejected_without_motion();
  insufficient_positive_limit_is_rejected_before_motion();
  return 0;
}
