"""Align the chassis, capture one shoulder-frame goal, then finish from feedback.

An instance is one operator-enabled session. Once a goal is captured, camera
visibility no longer affects it; completion or loss of arm feedback is terminal.
"""

from __future__ import annotations

import math

from .arm_tracking import ArmTracker, SHOULDER_FRAME, _array, _number


ALIGNMENT_TOLERANCE_M = .002
SETTLE_TIME_S = .3
GOAL_TOLERANCE_M = .002
GOAL_PITCH_TOLERANCE_RAD = .01


class ArmApproach:
    """Capture a target only after fresh centered frames follow a zero-Y hold.

``evaluate`` accepts the same arguments as ``ArmTracker.evaluate``. Image
receipt timestamps must describe the source image, never the command emission
time. After capture, only measured arm pose and its freshness are consulted.
"""

    def __init__(self, config, capture_after=-math.inf):
        self._tracker = ArmTracker(config, mode="depth")
        self.config = self._tracker.config
        self.mode = "approach"
        if capture_after != -math.inf:
            capture_after = _number(capture_after, "capture_after")
            if capture_after < 0:
                raise ValueError("capture_after must be nonnegative or -inf")
        self.capture_after = capture_after
        self.phase = "aligning"
        self._settle_started_at = None
        self._last_now = None
        self._last_pose_received_at = None
        self._count = 0
        self._goal = None
        self._goal_pitch = None
        self._goal_capture_timestamp_ms = None
        self._goal_capture_received_at = None
        self._terminal_reason = None

    def _result(self, reason, target=None, command_kind=None):
        return {
            "valid": target is not None,
            "reason": reason,
            "frame_id": SHOULDER_FRAME,
            "target_m": None if target is None else list(target),
            "consecutive_valid_frames": self._count,
            "tracking_mode": self.mode,
            "phase": self.phase,
            "command_kind": command_kind,
            "goal_m": None if self._goal is None else list(self._goal),
            "goal_capture_timestamp_ms": self._goal_capture_timestamp_ms,
            "goal_capture_received_at": self._goal_capture_received_at,
        }

    def invalidate(self, reason):
        """Reset acquisition, or permanently cancel a captured goal."""
        if self._terminal_reason is not None:
            return self._result(self._terminal_reason)
        self._tracker.invalidate(reason)
        self._count = 0
        self._settle_started_at = None
        if self._goal is None:
            self.phase = "aligning"
        else:
            self.phase = "cancelled"
            self._terminal_reason = reason
        return self._result(reason)

    def _approach(self, pose):
        error = math.hypot(self._goal[0] - pose[0], self._goal[2] - pose[1])
        pitch_error = abs(math.remainder(pose[2] - self._goal_pitch, 2. * math.pi))
        if (error <= GOAL_TOLERANCE_M + 1e-12
                and pitch_error <= GOAL_PITCH_TOLERANCE_RAD + 1e-12):
            self.phase = "complete"
            self._terminal_reason = "goal_reached"
            return self._result(self._terminal_reason)
        self.phase = "approaching"
        return self._result("approaching", self._goal, "goal")

    def evaluate(self, detection, now_monotonic, frame_received_at,
                 arm_pose=None, pose_received_at=None):
        if self._terminal_reason is not None:
            return self._result(self._terminal_reason)
        try:
            now = _number(now_monotonic, "now_monotonic")
        except ValueError:
            return self.invalidate("invalid_time")
        if now < 0 or (self._last_now is not None and now < self._last_now):
            return self.invalidate("invalid_time")
        self._last_now = now

        # Pose is needed for live X/Z holds even with a fixed camera mount.
        if arm_pose is None or pose_received_at is None:
            return self.invalidate("missing_arm_pose")
        try:
            pose_received = _number(pose_received_at, "pose_received_at")
        except ValueError:
            return self.invalidate("invalid_pose_time")
        if pose_received < 0 or pose_received > now:
            return self.invalidate("invalid_pose_time")
        if (self._last_pose_received_at is not None
                and pose_received < self._last_pose_received_at):
            return self.invalidate("out_of_order_pose_time")
        if now - pose_received > self.config["max_pose_age_s"]:
            return self.invalidate("stale_arm_pose")
        try:
            pose = _array(arm_pose, (3,), "arm_pose")
            # NumPy would otherwise coerce a mixed bool/float pose into numbers.
            for coordinate in arm_pose:
                _number(coordinate, "arm_pose")
        except ValueError:
            return self.invalidate("invalid_arm_pose")
        self._last_pose_received_at = pose_received

        if self._goal is not None:
            return self._approach(pose)

        try:
            received = _number(frame_received_at, "frame_received_at")
        except ValueError:
            return self.invalidate("invalid_time")
        if received <= self.capture_after:
            return self.invalidate("pre_enable_frame")
        tracking = self._tracker.evaluate(
            detection, now, received, arm_pose=pose, pose_received_at=pose_received)
        self._count = tracking["consecutive_valid_frames"]
        if not tracking["valid"]:
            self._settle_started_at = None
            self.phase = "aligning"
            return self._result(tracking["reason"])

        x, y, z = tracking["target_m"]
        if abs(y) > ALIGNMENT_TOLERANCE_M:
            self._settle_started_at = None
            self.phase = "aligning"
            return self._result("aligning", [pose[0], y, pose[1]], "live")

        self.phase = "settling"
        if self._settle_started_at is None:
            # This is when zero Y is first commanded, not when its image arrived.
            self._settle_started_at = now
        if received <= self._settle_started_at + SETTLE_TIME_S:
            return self._result("settling", [pose[0], 0., pose[1]], "live")

        # Subtract the forward clearance once, in shoulder X; keep the measured
        # midpoint's vertical offset. A close target must not request a retreat.
        standoff = self.config["approach_standoff_m"]
        if standoff > 0.:
            x = max(float(pose[0]), x - standoff)
        self._goal = (x, 0., z)
        self._goal_pitch = float(pose[2])
        self._goal_capture_timestamp_ms = float(detection["device_timestamp_ms"])
        self._goal_capture_received_at = received
        return self._approach(pose)
