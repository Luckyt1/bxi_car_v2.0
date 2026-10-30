#!/usr/bin/env python3
"""Offline tests for one-shot approach acquisition and feedback-only completion."""

import math
from pathlib import Path
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/thirdparty"))
from vision.arm_approach import ArmApproach, SETTLE_TIME_S
from vision.arm_tracking import CAMERA_FRAME, transform_midpoint


def config(mount="fixed"):
    # Synthetic calibration for offline tests only.
    return {"calibrated": True, "camera_mount": mount,
            "rotation": np.eye(3).tolist(), "translation_m": [0., 0., 0.],
            "offset_below_m": .075, "min_valid_frames": 3,
            "max_frame_age_s": .25, "max_pose_age_s": .25, "max_target_jump_m": .05}


def detection(stamp, point=(.2, 0., .4)):
    return {"valid": True, "device_timestamp_ms": stamp,
            "position_3d": {"valid": True, "frame_id": CAMERA_FRAME,
                            "midpoint_m": list(point)}}


POSE = [.1, .15, 0.]


def frame(approach, now, point=(.2, 0., .4), received=None, pose=POSE):
    return approach.evaluate(detection(now * 1000., point), now,
                             now if received is None else received,
                             arm_pose=pose, pose_received_at=now)


def lock(approach, start=1., point=(.2, 0., .4), pose=POSE):
    for delta in (0., .05, .1, .2, .3, .4, .45):
        result = frame(approach, start + delta, point, pose=pose)
    if result["phase"] != "approaching":
        raise AssertionError(result)
    return result


class ArmApproachTests(unittest.TestCase):
    def test_acquires_three_frames_then_only_aligns_y(self):
        approach = ArmApproach(config())
        for now in (1., 1.05):
            result = frame(approach, now, point=(.2, .02, .4))
            self.assertFalse(result["valid"])
            self.assertEqual(result["reason"], "acquiring")
        result = frame(approach, 1.1, point=(.2, .02, .4))
        self.assertEqual(result["phase"], "aligning")
        self.assertEqual(result["command_kind"], "live")
        self.assertEqual(result["target_m"], [POSE[0], .02, POSE[1]])
        self.assertIsNone(result["goal_m"])

    def test_centered_y_is_zero_and_settle_uses_strict_image_time(self):
        approach = ArmApproach(config())
        for now in (1., 1.05, 1.1):
            result = frame(approach, now, point=(.2, .002, .4))
        self.assertEqual(result["phase"], "settling")
        self.assertEqual(result["target_m"], [POSE[0], 0., POSE[1]])
        frame(approach, 1.2)
        frame(approach, 1.3)
        deadline = 1.1 + SETTLE_TIME_S
        result = frame(approach, deadline, received=deadline)
        self.assertEqual(result["phase"], "settling")
        result = frame(approach, deadline + .01)
        self.assertEqual(result["phase"], "approaching")
        self.assertEqual(result["command_kind"], "goal")
        np.testing.assert_allclose(result["target_m"], [.2, 0., .325])

    def test_enable_barrier_excludes_old_and_exact_boundary_images(self):
        approach = ArmApproach(config(), capture_after=1.)
        for received in (.9, .95, 1.):
            result = frame(approach, 1., received=received)
            self.assertEqual(result["reason"], "pre_enable_frame")
            self.assertFalse(result["valid"])
            self.assertEqual(result["consecutive_valid_frames"], 0)
        for now in (1.01, 1.05):
            self.assertEqual(frame(approach, now)["reason"], "acquiring")
        self.assertEqual(frame(approach, 1.1)["phase"], "settling")

    def test_new_device_stamp_cannot_promote_pre_settle_source_image(self):
        approach = ArmApproach(config())
        for now in (1., 1.05, 1.1, 1.2, 1.3):
            frame(approach, now)
        result = frame(approach, 1.5, received=1.39)
        self.assertEqual(result["phase"], "settling")
        self.assertIsNone(result["goal_m"])
        result = frame(approach, 1.51, received=1.41)
        self.assertEqual(result["phase"], "approaching")
        self.assertEqual(result["goal_capture_received_at"], 1.41)

    def test_settle_begins_when_zero_y_is_commanded(self):
        approach = ArmApproach(config())
        for now, received in ((1., .8), (1.05, .85), (1.1, .9),
                              (1.2, 1.), (1.3, 1.1), (1.4, 1.2)):
            result = frame(approach, now, received=received)
        self.assertEqual(result["phase"], "settling")
        result = frame(approach, 1.5, received=1.41)
        self.assertEqual(result["phase"], "approaching")

    def test_invalid_detection_resets_settle_and_three_frame_gate(self):
        approach = ArmApproach(config())
        for now in (1., 1.05, 1.1, 1.2):
            frame(approach, now)
        result = approach.evaluate({"valid": False, "device_timestamp_ms": 1250.},
                                   1.25, 1.25, POSE, 1.25)
        self.assertFalse(result["valid"])
        self.assertIsNone(result["target_m"])
        self.assertEqual(result["phase"], "aligning")
        for now in (1.3, 1.35):
            self.assertEqual(frame(approach, now)["reason"], "acquiring")
        for now in (1.4, 1.5, 1.6, 1.65):
            self.assertEqual(frame(approach, now)["phase"], "settling")
        self.assertEqual(frame(approach, 1.71)["phase"], "approaching")

    def test_loss_of_center_resets_settle_without_moving_xz(self):
        approach = ArmApproach(config())
        for now in (1., 1.05, 1.1, 1.2):
            frame(approach, now)
        result = frame(approach, 1.25, point=(.2, -.003, .4))
        self.assertEqual(result["phase"], "aligning")
        self.assertEqual(result["target_m"], [POSE[0], -.003, POSE[1]])
        for now in (1.3, 1.4, 1.5, 1.59):
            self.assertEqual(frame(approach, now)["phase"], "settling")
        self.assertEqual(frame(approach, 1.61)["phase"], "approaching")

    def test_tool_transform_and_offset_are_captured_once(self):
        cfg = config("tool")
        cfg["rotation"] = [[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]
        cfg["translation_m"] = [0., 0., .0625]
        pose, point = [.1, .15, .2], [0., .01, .3]
        result = lock(ArmApproach(cfg), point=point, pose=pose)
        expected = transform_midpoint(cfg, point, pose)
        np.testing.assert_allclose(result["goal_m"], expected)

    def test_occlusion_and_missing_depth_do_not_change_committed_goal(self):
        approach = ArmApproach(config())
        committed = lock(approach)
        inputs = [None, {}, {"valid": False},
                  {"valid": True, "position_3d": None},
                  detection(-100., point=(4., 5., 6.))]
        for index, value in enumerate(inputs):
            now = 1.5 + index * .1
            result = approach.evaluate(value, now, math.nan, POSE, now)
            self.assertEqual(result["phase"], "approaching")
            self.assertEqual(result["target_m"], committed["goal_m"])
            self.assertEqual(result["goal_capture_timestamp_ms"],
                             committed["goal_capture_timestamp_ms"])
            self.assertEqual(result["goal_capture_received_at"],
                             committed["goal_capture_received_at"])

    def test_horizontal_depth_minus_standoff_is_frozen_despite_occlusion(self):
        cfg = config("tool")
        cfg.update(rotation=[[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]],
                   translation_m=[0., 0., .0625], offset_below_m=.0745,
                   approach_standoff_m=.06)
        approach = ArmApproach(cfg)
        result = lock(approach, point=[0., 0., .20], pose=[.14, .15, 0.])
        expected = [.28, 0., .138]  # Advance 14 cm; retain the 7.45 cm Z offset.
        np.testing.assert_allclose(result["goal_m"], expected, atol=1e-12)
        for index, point in enumerate((None, {}, detection(9000., [0., 0., .40]))):
            now = 1.5 + .1 * index
            result = approach.evaluate(point, now, None, [.20, .14, 0.], now)
            self.assertEqual(result["phase"], "approaching")
            np.testing.assert_allclose(result["target_m"], expected, atol=1e-12)
        result = approach.evaluate(None, 1.9, None, [.28, .138, 0.], 1.9)
        self.assertEqual(result["reason"], "goal_reached")

    def test_close_target_keeps_capture_x_and_still_moves_to_offset_height(self):
        cfg = config("tool")
        cfg.update(rotation=[[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]],
                   translation_m=[0., 0., .0625], offset_below_m=.0745,
                   approach_standoff_m=.06)
        for depth in (.03, .06):
            with self.subTest(depth=depth):
                result = lock(ArmApproach(cfg), point=[0., 0., depth], pose=[.14, .15, 0.])
                np.testing.assert_allclose(result["goal_m"], [.14, 0., .138], atol=1e-12)

    def test_standoff_is_along_shoulder_x_and_preserves_pitched_target_height(self):
        cfg = config("tool")
        cfg.update(rotation=[[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]],
                   translation_m=[0., 0., .0625], offset_below_m=.0745,
                   approach_standoff_m=.06)
        pitch = math.pi / 6
        result = lock(ArmApproach(cfg), point=[0., 0., .20], pose=[.14, .15, pitch])
        expected = [.14 + .20 * math.cos(pitch) - .0625 * math.sin(pitch) - .06,
                    0., .15 + .20 * math.sin(pitch) + .0625 * math.cos(pitch) - .0745]
        np.testing.assert_allclose(result["goal_m"], expected, atol=1e-12)

    def test_zero_or_absent_standoff_preserves_existing_goal_geometry(self):
        for standoff in (None, 0.):
            with self.subTest(standoff=standoff):
                cfg = config()
                if standoff is not None:
                    cfg["approach_standoff_m"] = standoff
                result = lock(ArmApproach(cfg), point=[.05, 0., .4])
                np.testing.assert_allclose(result["goal_m"], [.05, 0., .325], atol=1e-12)

    def test_result_mutation_cannot_change_frozen_goal(self):
        approach = ArmApproach(config())
        result = lock(approach)
        expected = list(result["goal_m"])
        result["target_m"][0] = 100.
        result["goal_m"][2] = 100.
        self.assertEqual(approach.evaluate(None, 1.5, None, POSE, 1.5)["target_m"], expected)

    def test_pose_faults_cancel_and_cannot_rearm(self):
        faults = [
            (None, 1.5, "missing_arm_pose"),
            (POSE, None, "missing_arm_pose"),
            (POSE, math.nan, "invalid_pose_time"),
            (POSE, 1.6, "invalid_pose_time"),
            (POSE, 1.44, "out_of_order_pose_time"),
            ([.1, math.nan, 0.], 1.5, "invalid_arm_pose"),
            ([.1, .15], 1.5, "invalid_arm_pose"),
            ([True, .15, 0.], 1.5, "invalid_arm_pose"),
        ]
        for pose, received, reason in faults:
            with self.subTest(reason=reason, pose=pose):
                approach = ArmApproach(config())
                committed = lock(approach)
                result = approach.evaluate(None, 1.5, None, pose, received)
                self.assertEqual(result["phase"], "cancelled")
                self.assertEqual(result["reason"], reason)
                self.assertIsNone(result["target_m"])
                result = frame(approach, 1.6)
                self.assertFalse(result["valid"])
                self.assertEqual(result["reason"], reason)
                self.assertEqual(result["goal_m"], committed["goal_m"])

    def test_stale_feedback_cancels_even_when_markers_visible(self):
        approach = ArmApproach(config())
        lock(approach)
        result = approach.evaluate(detection(1800.), 1.8, 1.8, POSE, 1.45)
        self.assertEqual(result["reason"], "stale_arm_pose")
        self.assertEqual(result["phase"], "cancelled")
        self.assertEqual(frame(approach, 1.9)["phase"], "cancelled")

    def test_time_reversal_or_invalid_time_cancels(self):
        for now in (1.44, math.nan, math.inf, True):
            with self.subTest(now=now):
                approach = ArmApproach(config())
                lock(approach)
                result = approach.evaluate(None, now, None, POSE, 1.44)
                self.assertEqual(result["reason"], "invalid_time")
                self.assertEqual(result["phase"], "cancelled")
                self.assertFalse(frame(approach, 1.6)["valid"])

    def test_completion_uses_combined_xz_error_and_never_recaptures(self):
        approach = ArmApproach(config())
        goal = lock(approach)["goal_m"]
        result = approach.evaluate(None, 1.5, None,
                                   [goal[0] + .0015, goal[2] + .0015, 0.], 1.5)
        self.assertEqual(result["phase"], "approaching")
        result = approach.evaluate(None, 1.6, None,
                                   [goal[0] + .001, goal[2] + .001, 0.], 1.6)
        self.assertFalse(result["valid"])
        self.assertEqual(result["phase"], "complete")
        self.assertEqual(result["reason"], "goal_reached")
        self.assertIsNone(result["target_m"])
        self.assertEqual(frame(approach, 1.7, point=(.4, 0., .6))["goal_m"], goal)
        self.assertEqual(approach.invalidate("operator_stop")["phase"], "complete")

    def test_completion_waits_for_pitch_after_xz_arrives(self):
        approach = ArmApproach(config())
        goal = lock(approach)["goal_m"]
        for now, pitch in ((1.5, .1), (1.6, .02)):
            result = approach.evaluate(None, now, None, [goal[0], goal[2], pitch], now)
            self.assertEqual(result["phase"], "approaching")
            self.assertTrue(result["valid"])
            self.assertEqual(result["target_m"], goal)
        result = approach.evaluate(None, 1.7, None, [goal[0], goal[2], .005], 1.7)
        self.assertEqual(result["reason"], "goal_reached")

    def test_completion_pitch_uses_captured_pose_and_wraps_at_pi(self):
        approach = ArmApproach(config())
        goal = lock(approach, pose=[.1, .15, math.pi - .002])["goal_m"]
        result = approach.evaluate(None, 1.5, None,
                                   [goal[0], goal[2], -math.pi + .03], 1.5)
        self.assertEqual(result["phase"], "approaching")
        result = approach.evaluate(None, 1.6, None,
                                   [goal[0], goal[2], -math.pi + .002], 1.6)
        self.assertEqual(result["reason"], "goal_reached")

    def test_explicit_invalidation_cancels_committed_approach(self):
        approach = ArmApproach(config())
        lock(approach)
        result = approach.invalidate("operator_stop")
        self.assertEqual(result["phase"], "cancelled")
        self.assertEqual(result["reason"], "operator_stop")
        self.assertEqual(frame(approach, 1.6)["reason"], "operator_stop")

    def test_exact_goal_tolerance_is_complete_despite_float_roundoff(self):
        approach = ArmApproach(config())
        goal = lock(approach)["goal_m"]
        result = approach.evaluate(None, 1.5, None,
                                   [goal[0] + .002, goal[2], 0.], 1.5)
        self.assertEqual(result["phase"], "complete")

    def test_new_instance_can_capture_new_goal(self):
        original = lock(ArmApproach(config()))["goal_m"]
        new = lock(ArmApproach(config(), capture_after=2.), start=2.01,
                   point=(.25, 0., .45))["goal_m"]
        self.assertNotEqual(original, new)
        np.testing.assert_allclose(new, [.25, 0., .375])

    def test_pose_is_required_before_capture_even_for_fixed_camera(self):
        approach = ArmApproach(config())
        result = approach.evaluate(detection(1000.), 1., 1.)
        self.assertEqual(result["reason"], "missing_arm_pose")
        self.assertEqual(result["phase"], "aligning")
        self.assertEqual(lock(approach, start=1.1)["phase"], "approaching")

    def test_capture_barrier_configuration_is_validated(self):
        for barrier in (None, "1", True, math.inf, math.nan, -.1):
            with self.subTest(barrier=barrier), self.assertRaises(ValueError):
                ArmApproach(config(), capture_after=barrier)


if __name__ == "__main__":
    unittest.main()
