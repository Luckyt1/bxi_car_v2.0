"""Validated camera-to-shoulder geometry and a hardware-free tracking gate.

All translations are metres and arm poses are ``[x_m, z_m, pitch_rad]`` from
measured forward kinematics. ``rotation`` and ``translation_m`` map camera
optical coordinates into the shoulder frame (fixed mount) or tool frame (tool
mount). Positive arm pitch rotates +X towards +Z, i.e. around shoulder -Y.
The depth-mode downward offset is applied in shoulder coordinates after either
transform. Pixel mode uses image error only. Pixel-depth mode also holds the
optical depth acquired at the beginning of each tracking session.

This module never publishes commands. The controller must independently hold
the enabled pitch (and X when depth control is off), motion limits and watchdog.
"""

from __future__ import annotations

import json
import math
from numbers import Real
from pathlib import Path

import numpy as np


CAMERA_FRAME = "gemini_color_optical_frame"
SHOULDER_FRAME = "arm_shoulder"
PIXEL_DEADBAND_PX = 8.
PIXEL_GAIN_M_PER_PX = .0001  # A control gain, not a pixel-to-distance calibration.
PIXEL_MIN_CORRECTION_M = .003  # Exceed the receiver's 2 mm target deadband.
PIXEL_MAX_CORRECTION_M = .010
PIXEL_MAX_JUMP_PX = 80.
PIXEL_MIN_UP_PROJECTION = math.cos(math.radians(30.))
DEPTH_DEADBAND_M = .005
_CONFIG_KEYS = {
    "calibrated", "camera_mount", "rotation", "translation_m", "offset_below_m",
    "min_valid_frames", "max_frame_age_s", "max_pose_age_s", "max_target_jump_m",
}
_OPTIONAL_CONFIG_KEYS = {"approach_standoff_m"}


def _number(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    try:
        value = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return value


def _array(value, shape, name):
    try:
        array = np.asarray(value)
        if array.shape != shape or array.dtype.kind not in "iuf":
            raise ValueError
        array = array.astype(np.float64)
        if not np.isfinite(array).all():
            raise ValueError
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must contain finite numbers with shape {shape}") from exc
    return array


def validate_config(config):
    """Return an independent validated config; refuse absent/fake calibration.

Setting ``calibrated`` true is an explicit operator assertion that the supplied
rigid extrinsics were measured. There is deliberately no identity fallback.
"""
    if not isinstance(config, dict):
        raise ValueError("arm tracking configuration must be a JSON object")
    if config.get("calibrated") is not True:
        raise ValueError(
            "camera extrinsics are not calibrated: measure rotation/translation_m "
            "for the selected camera_mount, then set calibrated=true")
    missing = _CONFIG_KEYS - config.keys()
    unknown = config.keys() - _CONFIG_KEYS - _OPTIONAL_CONFIG_KEYS
    if missing or unknown:
        raise ValueError(f"invalid arm tracking configuration keys: missing={sorted(missing)}, "
                         f"unknown={sorted(unknown)}")
    if config["camera_mount"] not in ("fixed", "tool"):
        raise ValueError("camera_mount must be 'fixed' or 'tool'")
    rotation = _array(config["rotation"], (3, 3), "rotation")
    if (not np.allclose(rotation.T @ rotation, np.eye(3), rtol=0., atol=1e-6)
            or not math.isclose(float(np.linalg.det(rotation)), 1., rel_tol=0., abs_tol=1e-6)):
        raise ValueError("rotation must be a proper orthonormal 3x3 rotation (determinant +1)")
    translation = _array(config["translation_m"], (3,), "translation_m")
    result = dict(config, rotation=rotation.tolist(), translation_m=translation.tolist())
    result["approach_standoff_m"] = _number(config.get("approach_standoff_m", 0.),
                                            "approach_standoff_m")
    if result["approach_standoff_m"] < 0:
        raise ValueError("approach_standoff_m must be nonnegative")
    for name in ("offset_below_m", "max_frame_age_s", "max_pose_age_s", "max_target_jump_m"):
        result[name] = _number(config[name], name)
        if result[name] <= 0:
            raise ValueError(f"{name} must be positive")
    for name in ("max_frame_age_s", "max_pose_age_s"):
        if result[name] > .25:
            raise ValueError(f"{name} must not exceed 0.25 seconds")
    count = config["min_valid_frames"]
    if isinstance(count, bool) or not isinstance(count, int) or count < 3:
        raise ValueError("min_valid_frames must be an integer of at least 3")
    return result


def load_config(path):
    """Read a JSON config and fail before a caller opens hardware or enables motion."""
    with Path(path).open(encoding="utf-8") as source:
        return validate_config(json.load(source))


def _transform(config, camera_point, arm_pose):
    point = _array(camera_point, (3,), "camera_point")
    if point[2] <= 0:
        raise ValueError("camera_point optical depth must be positive")
    with np.errstate(over="ignore", invalid="ignore"):
        transformed = np.asarray(config["rotation"]) @ point + config["translation_m"]
        if config["camera_mount"] == "tool":
            if arm_pose is None:
                raise ValueError("tool-mounted camera requires measured arm_pose [x_m, z_m, pitch_rad]")
            x, z, pitch = _array(arm_pose, (3,), "arm_pose")
            cosine, sine = math.cos(pitch), math.sin(pitch)
            rotation = np.array([[cosine, 0., -sine], [0., 1., 0.], [sine, 0., cosine]])
            transformed = rotation @ transformed + [x, 0., z]
        elif arm_pose is not None:
            _array(arm_pose, (3,), "arm_pose")
        transformed[2] -= config["offset_below_m"]
    if not np.isfinite(transformed).all():
        raise ValueError("camera transform produced a nonfinite target")
    return transformed


def transform_midpoint(config, camera_point, arm_pose=None):
    """Return the full shoulder target [x, y, z], with the vertical offset applied.

This pure geometry function cannot establish feedback age. Live callers with a
tool mount must use ``ArmTracker.evaluate`` with pose/frame receipt timestamps.
"""
    return _transform(validate_config(config), camera_point, arm_pose).tolist()


class ArmTracker:
    """Require fresh, unique camera frames and consecutive plausible targets.

``detection`` has the existing QR/ArUco JSON schema, including
``device_timestamp_ms`` and either ``position_3d`` (depth mode) or
``midpoint_px``/``image_size`` (pixel mode); pixel-depth requires both. All ``*_received_at`` values and
``now_monotonic`` use the same local monotonic clock; device timestamps serve
only as strictly increasing frame identities, never as the local clock.

Every invalid result clears acquisition and returns ``target_m=None``. A jump
starts a new candidate sequence and needs at least ``min_valid_frames`` nearby
frames before reacquisition; one outlier cannot re-anchor the next frame.
"""

    def __init__(self, config, mode="depth"):
        self.config = validate_config(config)
        if mode not in ("pixel", "pixel-depth", "depth"):
            raise ValueError("tracking mode must be 'pixel', 'pixel-depth' or 'depth'")
        self.mode = mode
        if mode != "depth":
            rotation = np.asarray(self.config["rotation"])
            if (self.config["camera_mount"] != "tool"
                    or float(rotation[0, 2]) < .95
                    or float(-rotation[2, 1]) < .95):
                raise ValueError("pixel tracking requires a forward-facing tool camera "
                                 "with image up aligned to tool up")
        self._candidate = None
        self._candidate_pixel = None
        self._candidate_image_size = None
        self._candidate_depth = None
        self._reference_depth_m = None
        self._pixel_details = {}
        self._count = 0
        self._last_now = None
        self._last_device_timestamp = None
        self._last_frame_received_at = None

    def _result(self, reason, target=None):
        return {"valid": target is not None, "reason": reason, "frame_id": SHOULDER_FRAME,
                "target_m": None if target is None else target.tolist(),
                "consecutive_valid_frames": self._count, "tracking_mode": self.mode,
                **self._pixel_details}

    def invalidate(self, reason):
        """Clear acquisition while retaining the current session's depth reference."""
        self._candidate = None
        self._candidate_pixel = None
        self._candidate_image_size = None
        self._candidate_depth = None
        self._pixel_details = {}
        self._count = 0
        return self._result(reason)

    def _pixel_target(self, detection, arm_pose):
        """Use bounded image-error feedback; the metres are a command, not range."""
        midpoint = _array(detection.get("midpoint_px"), (2,), "midpoint_px")
        size = _array(detection.get("image_size"), (2,), "image_size")
        if np.any(size <= 1) or np.any(size != np.floor(size)):
            raise ValueError("invalid image size")
        if np.any(midpoint < 0) or np.any(midpoint >= size):
            raise ValueError("midpoint is outside image")
        x, z, pitch = _array(arm_pose, (3,), "arm_pose")
        image_up = -np.asarray(self.config["rotation"])[:, 1]
        up_projection = math.sin(pitch) * image_up[0] + math.cos(pitch) * image_up[2]
        if up_projection < PIXEL_MIN_UP_PROJECTION:
            return None, midpoint, size
        reference = (size - 1.) / 2.
        error = reference - midpoint
        # Image left is shoulder/chassis +Y; image up is arm +Z.
        correction = [0. if abs(delta) <= PIXEL_DEADBAND_PX else
                      math.copysign(min(PIXEL_MAX_CORRECTION_M,
                                        max(PIXEL_MIN_CORRECTION_M,
                                            abs(delta) * PIXEL_GAIN_M_PER_PX)), delta)
                      for delta in error]
        self._pixel_details = {"pixel_reference_x": float(reference[0]),
                               "pixel_reference_y": float(reference[1]),
                               "pixel_error_x": float(error[0]),
                               "pixel_error_y": float(error[1]),
                               "y_correction_m": correction[0],
                               "z_correction_m": correction[1]}
        return np.array([x, correction[0], z + correction[1]]), midpoint, size

    def _depth_correction(self, depth, arm_pose):
        error = 0. if self._reference_depth_m is None else depth - self._reference_depth_m
        pitch = float(arm_pose[2])
        forward = np.asarray(self.config["rotation"])[:, 2]
        forward_projection = math.cos(pitch) * forward[0] - math.sin(pitch) * forward[2]
        if forward_projection <= 0:
            raise ValueError("camera forward must project onto shoulder +X")
        # Subtracting two metre values must not move an exact 5 mm error outside the deadband.
        correction = (0. if abs(error) <= DEPTH_DEADBAND_M + 1e-9 else
                      math.copysign(min(PIXEL_MAX_CORRECTION_M,
                                        max(PIXEL_MIN_CORRECTION_M,
                                            abs(error) / forward_projection)), error))
        self._pixel_details.update(depth_m=depth, depth_reference_m=self._reference_depth_m,
                                   depth_error_m=error, x_correction_m=correction)
        return correction

    def evaluate(self, detection, now_monotonic, frame_received_at,
                 arm_pose=None, pose_received_at=None):
        """Return only this evaluation's valid target or an explicit rejection reason."""
        self._pixel_details = {}
        try:
            now = _number(now_monotonic, "now_monotonic")
            received = _number(frame_received_at, "frame_received_at")
        except ValueError:
            return self.invalidate("invalid_time")
        if now < 0 or received < 0 or (self._last_now is not None and now < self._last_now):
            return self.invalidate("invalid_time")
        self._last_now = now
        if not isinstance(detection, dict):
            return self.invalidate("invalid_detection")
        try:
            stamp = _number(detection.get("device_timestamp_ms"), "device_timestamp_ms")
        except ValueError:
            return self.invalidate("invalid_frame_timestamp")
        if stamp < 0:
            return self.invalidate("invalid_frame_timestamp")
        if self._last_device_timestamp is not None and stamp <= self._last_device_timestamp:
            return self.invalidate("duplicate_or_out_of_order_frame")
        self._last_device_timestamp = stamp
        if received > now:
            return self.invalidate("future_frame")
        if now - received > self.config["max_frame_age_s"]:
            return self.invalidate("stale_frame")
        if self._last_frame_received_at is not None:
            if received <= self._last_frame_received_at:
                return self.invalidate("out_of_order_frame_time")
            if received - self._last_frame_received_at > self.config["max_frame_age_s"]:
                self.invalidate("frame_gap")
        self._last_frame_received_at = received
        if detection.get("valid") is not True:
            return self.invalidate("invalid_detection")
        if self.mode != "pixel":
            position = detection.get("position_3d")
            if not isinstance(position, dict) or position.get("valid") is not True:
                return self.invalidate("invalid_detection")
            if position.get("frame_id") != CAMERA_FRAME:
                return self.invalidate("wrong_camera_frame")
        if self.config["camera_mount"] == "tool":
            if arm_pose is None or pose_received_at is None:
                return self.invalidate("missing_arm_pose")
            try:
                pose_received = _number(pose_received_at, "pose_received_at")
            except ValueError:
                return self.invalidate("invalid_pose_time")
            if pose_received < 0 or pose_received > now:
                return self.invalidate("invalid_pose_time")
            if now - pose_received > self.config["max_pose_age_s"]:
                return self.invalidate("stale_arm_pose")
        try:
            depth = None
            if self.mode != "depth":
                target, pixel, size = self._pixel_target(detection, arm_pose)
                if target is None:
                    return self.invalidate("pixel_camera_not_upright")
                if self.mode == "pixel-depth":
                    depth = float(_array(position.get("midpoint_m"), (3,), "midpoint_m")[2])
                    if depth <= 0:
                        raise ValueError("optical depth must be positive")
                    target[0] += self._depth_correction(depth, arm_pose)
            else:
                target = _transform(self.config, position.get("midpoint_m"), arm_pose)
        except ValueError:
            return self.invalidate("invalid_geometry")
        jumped = (self._candidate is not None and
                  np.linalg.norm(target - self._candidate) > self.config["max_target_jump_m"])
        if self.mode != "depth":
            jumped = jumped or (self._candidate_pixel is not None and
                                (not np.array_equal(size, self._candidate_image_size)
                                 or np.linalg.norm(pixel - self._candidate_pixel)
                                 > PIXEL_MAX_JUMP_PX))
            self._candidate_pixel, self._candidate_image_size = pixel, size
            if self.mode == "pixel-depth":
                jumped = jumped or (self._candidate_depth is not None and
                                    abs(depth - self._candidate_depth)
                                    > self.config["max_target_jump_m"])
                self._candidate_depth = depth
        self._count = (1 if self._candidate is None or jumped else
                       min(self._count + 1, self.config["min_valid_frames"]))
        self._candidate = target
        if jumped:
            return self._result("target_jump")
        if self._count < self.config["min_valid_frames"]:
            return self._result("acquiring")
        if self.mode == "pixel-depth" and self._reference_depth_m is None:
            self._reference_depth_m = depth
            self._pixel_details["depth_reference_m"] = depth
        return self._result("tracking", target)
