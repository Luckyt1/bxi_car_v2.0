#include "chassis/swerve_kinematics.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>

namespace
{
bool near(double left, double right, double epsilon = 1e-9)
{
  return std::abs(left - right) < epsilon;
}
}  // namespace

int main()
{
  constexpr double pi = 3.14159265358979323846;
  const std::array<double, 4> straight{};

  const auto forward = chassis::swerve_commands(0.1, 0.0, 0.0, 0.46, 0.40, 0.13, 1.0, straight);
  for (const auto & wheel : forward) {
    assert(near(wheel.steering_rad, 0.0));
    assert(near(wheel.drive_rpm, 1.0));
  }

  const auto reverse = chassis::swerve_commands(-0.1, 0.0, 0.0, 0.46, 0.40, 0.13, 1.0, straight);
  for (const auto & wheel : reverse) {
    assert(near(wheel.steering_rad, 0.0));
    assert(near(wheel.drive_rpm, -1.0));
  }

  const auto rotate = chassis::swerve_commands(0.0, 0.0, 0.5, 0.46, 0.40, 0.13, 1.0, straight);
  assert(near(rotate[0].steering_rad, std::atan2(-0.23, 0.20)));
  assert(near(rotate[1].steering_rad, std::atan2(0.23, 0.20)));
  assert(near(rotate[2].steering_rad, std::atan2(-0.23, 0.20)));
  assert(near(rotate[3].steering_rad, std::atan2(0.23, 0.20)));
  assert(rotate[0].drive_rpm < 0.0 && rotate[1].drive_rpm < 0.0);
  assert(rotate[2].drive_rpm > 0.0 && rotate[3].drive_rpm > 0.0);
  for (const auto & wheel : rotate) {
    assert(near(std::abs(wheel.drive_rpm), 1.0));
  }

  // At the calibrated midpoint, positive travel points to chassis-right (-Y).
  // A +pi/2 steering mounting offset must therefore map +X to chassis-forward.
  const auto mounted_forward = chassis::swerve_commands(
    0.1, 0.0, 0.0, 0.46, 0.40, 0.13, 1.0, straight, pi / 2.0);
  for (const auto & wheel : mounted_forward) {
    assert(near(wheel.steering_rad, pi / 2.0));
    assert(near(wheel.drive_rpm, 1.0));
  }

  const auto mounted_reverse = chassis::swerve_commands(
    -0.1, 0.0, 0.0, 0.46, 0.40, 0.13, 1.0, straight, pi / 2.0);
  for (const auto & wheel : mounted_reverse) {
    assert(near(wheel.steering_rad, -pi / 2.0));
    assert(near(wheel.drive_rpm, 1.0));
  }

  const auto mounted_rotate = chassis::swerve_commands(
    0.0, 0.0, 0.5, 0.46, 0.40, 0.13, 1.0, straight, pi / 2.0);
  const std::array<double, 4> desired_x{-0.20, -0.20, 0.20, 0.20};
  const std::array<double, 4> desired_y{0.23, -0.23, -0.23, 0.23};
  for (std::size_t index = 0; index < mounted_rotate.size(); ++index) {
    const double direction = mounted_rotate[index].drive_rpm < 0.0 ? pi : 0.0;
    const double physical_angle = mounted_rotate[index].steering_rad - pi / 2.0 + direction;
    const double desired_angle = std::atan2(desired_y[index], desired_x[index]);
    assert(near(chassis::wrap_to_pi(physical_angle - desired_angle), 0.0));
    assert(near(std::abs(mounted_rotate[index].drive_rpm), 1.0));
  }

  const auto full_remote_command = chassis::swerve_commands(
    0.6, 0.0, 0.4, 0.46, 0.40, 0.13, 105.0, straight, pi / 2.0);
  const double fastest_required_rpm = std::hypot(0.68, 0.092) * 60.0 / (pi * 0.13);
  assert(fastest_required_rpm < 105.0);
  assert(near(std::abs(full_remote_command[2].drive_rpm), fastest_required_rpm));
  assert(near(std::abs(full_remote_command[3].drive_rpm), fastest_required_rpm));

  const auto stopped = chassis::swerve_commands(
    0.0, 0.0, 0.0, 0.46, 0.40, 0.13, 1.0,
    std::array<double, 4>{0.1, -0.2, pi / 4.0, -pi / 4.0});
  assert(near(stopped[0].steering_rad, 0.1) && near(stopped[0].drive_rpm, 0.0));
  assert(near(stopped[1].steering_rad, -0.2) && near(stopped[1].drive_rpm, 0.0));

  // Mechanical endpoints are not adjacent across the +/-pi boundary.
  std::array<chassis::SteeringRange, 4> ranges;
  ranges.fill({-1.45, 1.45});
  std::array<double, 4> actual;
  actual.fill(-1.4);
  const auto limited = chassis::swerve_commands(
    std::cos(1.4), std::sin(1.4), 0, 0.46, 0.40, 0.13, 105, actual, 0, ranges);
  for (const auto & wheel : limited) {
    assert(near(wheel.steering_rad, 1.4));
    assert(wheel.drive_rpm > 0);  // Must use the long but reachable mechanical path.
  }
  bool rejected = false;
  try {
    (void)chassis::swerve_commands(0, 1, 0, 0.46, 0.40, 0.13, 105, actual, 0, ranges);
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  assert(rejected);  // Never silently clip an unreachable angle and then drive.

  std::array<double, 4> previous;
  previous.fill(pi / 2);
  for (const double angle : {pi / 2 - 0.001, pi / 2 + 0.001}) {
    const auto stable = chassis::swerve_commands(
      std::cos(angle), std::sin(angle), 0, 0.46, 0.40, 0.13, 105, straight, 0, {}, previous);
    for (const auto & wheel : stable) {
      assert(wheel.steering_rad > 0 && wheel.drive_rpm > 0);
    }
  }
  // Sweep reachable candidates and verify wheel velocity direction is preserved.
  for (int sample = -314; sample <= 314; ++sample) {
    const double angle = sample / 100.0;
    const auto sweep = chassis::swerve_commands(
      std::cos(angle), std::sin(angle), 0, 0.46, 0.40, 0.13, 105, actual);
    for (const auto & wheel : sweep) {
      assert(wheel.steering_rad >= -pi && wheel.steering_rad <= pi);
      const double physical = wheel.steering_rad + (wheel.drive_rpm < 0 ? pi : 0);
      assert(near(chassis::wrap_to_pi(physical - angle), 0));
    }
  }
  assert(near(chassis::slew_towards(0, 1, 0.01), 0.01));
  assert(near(chassis::slew_towards(0, -1, 0.01), -0.01));
}
