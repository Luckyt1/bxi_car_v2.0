#pragma once

#include "example/app_interface.h"
#include "example/arm_visual_tracking.h"

#include <chrono>
#include <functional>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace chassis
{
// Called by the hardware owner's serial executor, before advancing arm feedback.
class AppArmControl
{
public:
  using Clock = std::chrono::steady_clock;

  AppArmControl(
    rclcpp::Node & node, ArmController & arm, ArmVisualTracking & tracker,
    std::function<bool(Clock::time_point)> fresh_command)
  : node_(node), arm_(arm), tracker_(tracker), fresh_command_(std::move(fresh_command)) {}

  void command(int value, Clock::time_point now = Clock::now())
  {
    if (value == 109) {stop(now); return;}
    if (!fresh_command_(now)) {
      RCLCPP_WARN(node_.get_logger(), "App arm command ignored: no fresh gateway control heartbeat");
      return;
    }
    if (value == 107 || value == 108) {
      // One tap grants a bounded jog. Further taps cannot renew or reverse it.
      if (!arm_.has_fourth_axis() || fourth_deadline_) {return;}
      arm_.set_fourth_axis_jog(value == 107 ? 1 : -1);
      fourth_deadline_ = now + std::chrono::milliseconds(100);
      stop_latched_ = false;
      return;
    }
    const auto target = app_arm_target(arm_.actual_pose(), value);
    if (!target) {return;}
    // Reuse IK/limits/visual-ownership validation and the existing staged PP path.
    // Rejected/busy steps are discarded, never queued for later.
    const auto result = node_.set_parameter(rclcpp::Parameter(
      kYiyouPoseParameter, std::vector<double>{target->x, target->z, target->pitch}));
    if (result.successful) {
      stop_latched_ = false;
    } else {
      RCLCPP_WARN(node_.get_logger(), "App arm step rejected: %s", result.reason.c_str());
    }
  }

  void stop(Clock::time_point now = Clock::now())
  {
    if (stop_latched_ && !node_.get_parameter(kArmTrackingEnabledParameter).as_bool() &&
      !arm_.position_command_pending() && arm_.fourth_axis_jog() == 0)
    {
      // Pending PP only describes the handshake. An external parameter target
      // can be unapplied or still moving after that handshake has finished.
      const auto actual = arm_.actual_pose();
      if (arm_.planar_halt_pending() ||
        node_.get_parameter(kYiyouPoseParameter).as_double_array() ==
        std::vector<double>{actual.x, actual.z, actual.pitch}) {return;}
    }
    stop_latched_ = false;
    fourth_deadline_.reset();
    // Consume a committed target as well as cancelling an in-flight PP group,
    // so the normal parameter path cannot resume a stopped App action.
    const auto result = node_.set_parameter(rclcpp::Parameter(kArmTrackingEnabledParameter, false));
    if (!result.successful) {throw std::runtime_error(result.reason);}
    tracker_.stop_tracking(now);
    if (arm_.has_fourth_axis()) {arm_.set_fourth_axis_jog(0);}
    stop_latched_ = true;
  }

  void before_update(Clock::time_point now = Clock::now())
  {
    if (fourth_deadline_ && now >= *fourth_deadline_) {
      arm_.set_fourth_axis_jog(0);
      fourth_deadline_.reset();
    }
  }

private:
  rclcpp::Node & node_;
  ArmController & arm_;
  ArmVisualTracking & tracker_;
  std::function<bool(Clock::time_point)> fresh_command_;
  std::optional<Clock::time_point> fourth_deadline_;
  bool stop_latched_{false};
};
}  // namespace chassis
