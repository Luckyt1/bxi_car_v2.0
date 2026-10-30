#!/usr/bin/env python3
"""Camera/arm transform and tracking gating checks, without ROS or hardware."""

import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "arm_tracking", ROOT / "src/thirdparty/vision/arm_tracking.py")
tracking = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tracking)


def config(mount="fixed"):
    # Synthetic calibration for offline tests only, never a robot calibration.
    return {"calibrated": True, "camera_mount": mount,
            "rotation": np.eye(3).tolist(), "translation_m": [0., 0., 0.],
            "offset_below_m": .075, "min_valid_frames": 3,
            "max_frame_age_s": .25, "max_pose_age_s": .25, "max_target_jump_m": .05}


def detection(stamp=0., point=(.2, .1, .4)):
    return {"valid": True, "device_timestamp_ms": stamp,
            "position_3d": {"valid": True, "frame_id": tracking.CAMERA_FRAME,
                            "midpoint_m": list(point)}}


class GeometryTests(unittest.TestCase):
    def test_fixed_camera_maps_all_axes_then_offsets_shoulder_vertical(self):
        cfg = config()
        cfg["rotation"] = [[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]
        cfg["translation_m"] = [.1, .2, .3]
        np.testing.assert_allclose(
            tracking.transform_midpoint(cfg, [.02, .04, .5]), [.6, .18, .185])

    def test_offset_stays_vertical_for_different_fixed_camera_rotations(self):
        for angle in (0., .4, math.pi / 2, math.pi):
            cfg = config()
            c, s = math.cos(angle), math.sin(angle)
            cfg["rotation"] = [[c, 0., s], [0., 1., 0.], [-s, 0., c]]
            raw = np.asarray(cfg["rotation"]) @ [.1, -.1, .5]
            target = tracking.transform_midpoint(cfg, [.1, -.1, .5])
            np.testing.assert_allclose(raw - target, [0., 0., .075], atol=1e-12)

    def test_tool_mount_uses_measured_pose_with_positive_pitch_toward_up(self):
        cfg = config("tool")
        cfg["translation_m"] = [.1, .02, .03]
        # At +90deg arm pitch, tool +X maps shoulder +Z; tool +Z maps shoulder -X.
        np.testing.assert_allclose(
            tracking.transform_midpoint(cfg, [.2, .1, .4], [.6, .5, math.pi / 2]),
            [.17, .12, .725], atol=1e-12)

    def test_offset_remains_vertical_as_arm_pitch_changes(self):
        cfg = config("tool")
        point = np.array([.2, -.1, .4])
        for pitch in (-1.2, 0., .7, math.pi / 2):
            c, s = math.cos(pitch), math.sin(pitch)
            tool_to_shoulder = np.array([[c, 0., -s], [0., 1., 0.], [s, 0., c]])
            raw = tool_to_shoulder @ point + [.4, 0., .3]
            target = tracking.transform_midpoint(cfg, point, [.4, .3, pitch])
            np.testing.assert_allclose(raw - target, [0., 0., .075], atol=1e-12)

    def test_stationary_target_is_invariant_under_tool_camera_motion(self):
        cfg = config("tool")
        cfg["rotation"] = [[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]
        cfg["translation_m"] = [.05, .01, .03]
        midpoint = np.array([1.1, .04, .5])
        expected = midpoint - [0., 0., .075]
        for x, z, pitch in ((.4, .3, -.4), (.5, .4, 0.), (.3, .2, .7)):
            c, s = math.cos(pitch), math.sin(pitch)
            rotation = np.array([[c, 0., -s], [0., 1., 0.], [s, 0., c]])
            tool_point = rotation.T @ (midpoint - [x, 0., z])
            camera_point = np.asarray(cfg["rotation"]).T @ (tool_point - cfg["translation_m"])
            self.assertGreater(camera_point[2], 0.)
            np.testing.assert_allclose(
                tracking.transform_midpoint(cfg, camera_point, [x, z, pitch]), expected)

    def test_transform_overflow_is_rejected(self):
        cfg = config()
        cfg["translation_m"] = [0., 0., 1.5e308]
        with self.assertRaisesRegex(ValueError, "nonfinite target"):
            tracking.transform_midpoint(cfg, [0., 0., 1.5e308])

    def test_tool_mount_does_not_supply_missing_pose(self):
        with self.assertRaisesRegex(ValueError, "requires measured arm_pose"):
            tracking.transform_midpoint(config("tool"), [0., 0., 1.])

    def test_no_invalid_camera_point_or_pose(self):
        for point in ([0., 1.], [[0., 0., 1.]], [0., 0., 0.], [0., 0., -1.],
                      [0., 0., math.nan], [math.inf, 0., 1.], ["0", "0", "1"], None):
            with self.subTest(point=point), self.assertRaises(ValueError):
                tracking.transform_midpoint(config(), point)
        for pose in ([0., 0.], [0., 0., math.nan], [0., math.inf, 0.], [0., 0., "0"]):
            with self.subTest(pose=pose), self.assertRaises(ValueError):
                tracking.transform_midpoint(config("tool"), [0., 0., 1.], pose)

    def test_confirmed_mount_height_and_vertical_offset_are_composed(self):
        path = ROOT / "src/config/arm_tracking.json"
        default = tracking.load_config(path)
        self.assertEqual(default["camera_mount"], "tool")
        self.assertEqual(default["offset_below_m"], .0745)
        self.assertEqual(default["approach_standoff_m"], .04)
        # A point on the optical axis is 62.5 mm above a horizontal TCP;
        # placing the TCP 74.5 mm below that point requires a 12 mm descent.
        # Approach clearance must not affect the shared depth-mode transform.
        target = tracking.transform_midpoint(default, [0., 0., 1.], [.2, .3, 0.])
        np.testing.assert_allclose(target, [1.2, 0., .288], atol=1e-12)
        default["calibrated"] = False
        with self.assertRaisesRegex(ValueError, "extrinsics are not calibrated"):
            tracking.validate_config(default)

    def test_load_and_validation_make_independent_config(self):
        cfg = config()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            path.write_text(json.dumps(cfg), encoding="utf-8")
            validated = tracking.load_config(path)
        validated["rotation"][0][0] = 2.
        self.assertEqual(cfg["rotation"][0][0], 1.)

    def test_approach_standoff_is_optional_and_nonnegative(self):
        cfg = config()
        self.assertEqual(tracking.validate_config(cfg)["approach_standoff_m"], 0.)
        self.assertNotIn("approach_standoff_m", cfg)
        for standoff in (0., .06):
            cfg["approach_standoff_m"] = standoff
            self.assertEqual(tracking.validate_config(cfg)["approach_standoff_m"], standoff)

    def test_reject_non_rigid_and_reflected_transforms(self):
        for rotation in (np.diag([2., 1., 1.]).tolist(), np.diag([-1., 1., 1.]).tolist(),
                         [[1., .1, 0.], [0., 1., 0.], [0., 0., 1.]],
                         np.eye(4).tolist(), [[math.nan] * 3] * 3, None):
            cfg = config()
            cfg["rotation"] = rotation
            with self.subTest(rotation=rotation), self.assertRaises(ValueError):
                tracking.validate_config(cfg)
        for translation in ([0., 0.], [0., 0., math.inf], [[0., 0., 0.]], None):
            cfg = config()
            cfg["translation_m"] = translation
            with self.subTest(translation=translation), self.assertRaises(ValueError):
                tracking.validate_config(cfg)

    def test_reject_unsafe_or_malformed_configuration(self):
        invalid = {"calibrated": [False, 1, "true"], "camera_mount": ["unknown", None],
                   "min_valid_frames": [0, 2, 3., True],
                   "offset_below_m": [0., -1., math.nan, True, ".075"],
                   "approach_standoff_m": [-.01, math.nan, math.inf, True, ".06", None],
                   "max_frame_age_s": [0., .251, math.inf],
                   "max_pose_age_s": [-.1, .3, math.nan],
                   "max_target_jump_m": [0., -1., math.inf, 10 ** 1000]}
        for field, values in invalid.items():
            for value in values:
                cfg = config()
                cfg[field] = value
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    tracking.validate_config(cfg)
        for cfg in (None, {}, dict(config(), typo=True)):
            with self.assertRaises(ValueError):
                tracking.validate_config(cfg)
        cfg = config()
        del cfg["translation_m"]
        with self.assertRaisesRegex(ValueError, "missing"):
            tracking.validate_config(cfg)


class TrackerTests(unittest.TestCase):
    def setUp(self):
        self.tracker = tracking.ArmTracker(config())
        self.tick = 0

    def step(self, point=(.2, .1, .4), **kwargs):
        self.tick += 1
        now = 10. + self.tick * .03
        record = kwargs.pop("record", detection(self.tick * 30., point))
        return self.tracker.evaluate(record, kwargs.pop("now_monotonic", now),
                                     kwargs.pop("frame_received_at", now - .01), **kwargs)

    def acquire(self):
        for _ in range(3):
            result = self.step()
        self.assertTrue(result["valid"])
        return result

    def assert_rejected(self, result, reason):
        self.assertFalse(result["valid"])
        self.assertIsNone(result["target_m"])
        self.assertEqual(result["reason"], reason)

    def test_three_unique_frames_are_required_and_output_is_latest(self):
        for count in (1, 2):
            result = self.step()
            self.assert_rejected(result, "acquiring")
            self.assertEqual(result["consecutive_valid_frames"], count)
        result = self.step()
        self.assertTrue(result["valid"])
        self.assertEqual(result["frame_id"], "arm_shoulder")
        np.testing.assert_allclose(result["target_m"], [.2, .1, .325])
        result = self.step(point=(.2, .1, .42))
        np.testing.assert_allclose(result["target_m"], [.2, .1, .345])

    def test_detection_loss_clears_output_and_requires_reacquisition(self):
        self.acquire()
        for key in ("valid", "position_3d"):
            invalid = detection((self.tick + 1) * 30.)
            if key == "valid":
                invalid["valid"] = False
            else:
                invalid["position_3d"]["valid"] = False
            self.assert_rejected(self.step(record=invalid), "invalid_detection")
            self.assert_rejected(self.step(), "acquiring")
            self.assert_rejected(self.step(), "acquiring")
            self.assertTrue(self.step()["valid"])

    def test_jump_cannot_reanchor_on_next_frame(self):
        self.acquire()
        point = (.4, .1, .4)
        self.assert_rejected(self.step(point), "target_jump")
        self.assert_rejected(self.step(point), "acquiring")
        self.assertTrue(self.step(point)["valid"])

    def test_repeated_unstable_jumps_never_acquire(self):
        self.acquire()
        for index in range(5):
            self.assert_rejected(self.step((.4 + index * .1, .1, .4)), "target_jump")

    def test_duplicate_frame_invalidates_without_bypassing_gate(self):
        self.acquire()
        self.assert_rejected(self.step(record=detection(90.)), "duplicate_or_out_of_order_frame")
        self.assert_rejected(self.step(), "acquiring")
        self.assert_rejected(self.step(), "acquiring")
        self.assertTrue(self.step()["valid"])

    def test_frame_timestamp_must_be_valid_strictly_increasing(self):
        for stamp in (None, -1., math.nan, math.inf, True, "30"):
            self.assert_rejected(self.step(record=detection(stamp)), "invalid_frame_timestamp")
        self.acquire()
        self.assert_rejected(self.step(record=detection(0.)), "duplicate_or_out_of_order_frame")

    def test_stale_or_future_frame_has_no_target(self):
        self.acquire()
        self.assert_rejected(self.step(frame_received_at=1.), "stale_frame")
        self.assert_rejected(self.step(frame_received_at=100.), "future_frame")

    def test_frame_gap_requires_new_consecutive_frames(self):
        self.acquire()
        self.tick += 30
        self.assert_rejected(self.step(), "acquiring")
        self.assert_rejected(self.step(), "acquiring")
        self.assertTrue(self.step()["valid"])

    def test_monotonic_times_reject_nonfinite_or_backward_values(self):
        for value in (None, -1., math.nan, math.inf, True):
            self.assert_rejected(self.step(now_monotonic=value), "invalid_time")
        self.acquire()
        self.assert_rejected(self.step(now_monotonic=1., frame_received_at=.9), "invalid_time")
        self.assert_rejected(self.step(frame_received_at=math.nan), "invalid_time")

    def test_receipt_time_must_increase_even_with_new_device_timestamp(self):
        self.step()
        self.assert_rejected(self.step(frame_received_at=10.02), "out_of_order_frame_time")

    def test_wrong_frame_or_invalid_3d_point_clears_output(self):
        self.acquire()
        invalid = detection((self.tick + 1) * 30.)
        invalid["position_3d"]["frame_id"] = "wrong_frame"
        self.assert_rejected(self.step(record=invalid), "wrong_camera_frame")
        for point in ((0., 0., 0.), (0., 0., -1.), (0., 0., math.nan), (math.inf, 0., .4), (0.,)):
            self.assert_rejected(self.step(point), "invalid_geometry")

    def test_tool_mount_requires_fresh_pose_and_frame(self):
        self.tracker = tracking.ArmTracker(config("tool"))
        self.assert_rejected(self.step(), "missing_arm_pose")
        self.assert_rejected(self.step(arm_pose=[.4, .3, 0.]), "missing_arm_pose")
        self.assert_rejected(self.step(arm_pose=[.4, .3, 0.], pose_received_at=1.), "stale_arm_pose")
        for stamp in (math.nan, math.inf, -1., 100.):
            self.assert_rejected(self.step(arm_pose=[.4, .3, 0.], pose_received_at=stamp),
                                 "invalid_pose_time")
        for count in (1, 2, 3):
            now = 10. + (self.tick + 1) * .03
            result = self.step(arm_pose=[.4, .3, 0.], pose_received_at=now - .02)
            self.assertEqual(result["valid"], count == 3)
        np.testing.assert_allclose(result["target_m"], [.6, .1, .625])

    def test_stale_pose_clears_an_acquired_tool_target(self):
        self.tracker = tracking.ArmTracker(config("tool"))
        for _ in range(3):
            now = 10. + (self.tick + 1) * .03
            result = self.step(arm_pose=[.4, .3, 0.], pose_received_at=now - .01)
        self.assertTrue(result["valid"])
        self.assert_rejected(self.step(arm_pose=[.4, .3, 0.], pose_received_at=1.), "stale_arm_pose")

    def test_invalid_tool_pose_has_no_target(self):
        self.tracker = tracking.ArmTracker(config("tool"))
        self.assert_rejected(self.step(arm_pose=[.4, .3, math.nan], pose_received_at=10.),
                             "invalid_geometry")

    def test_caller_cannot_mutate_calibration_after_construction(self):
        cfg = config()
        tracker = tracking.ArmTracker(cfg)
        cfg["translation_m"][2] = 100.
        for i in range(3):
            result = tracker.evaluate(detection(i), 1. + i * .03, .99 + i * .03)
        np.testing.assert_allclose(result["target_m"], [.2, .1, .325])


class PixelTrackerTests(unittest.TestCase):
    def setUp(self):
        self.config = tracking.load_config(ROOT / "src/config/arm_tracking.json")
        self.tracker = tracking.ArmTracker(self.config, mode="pixel")
        self.tick = 0

    def step(self, midpoint=(319.5, 239.5), **kwargs):
        self.tick += 1
        now = 10. + self.tick * .03
        record = kwargs.pop("record", {"valid": True,
                            "device_timestamp_ms": self.tick * 30.,
                            "midpoint_px": midpoint, "image_size": [640, 480]})
        pose = kwargs.pop("arm_pose", [.2, .3, 0.])
        return self.tracker.evaluate(record, kwargs.pop("now_monotonic", now),
                                     kwargs.pop("frame_received_at", now - .01),
                                     pose, kwargs.pop("pose_received_at", now - .01))

    def acquire(self, midpoint=(319.5, 239.5), **kwargs):
        for _ in range(3):
            result = self.step(midpoint, **kwargs)
        self.assertTrue(result["valid"], result)
        return result

    def assert_rejected(self, result, reason):
        self.assertFalse(result["valid"])
        self.assertIsNone(result["target_m"])
        self.assertEqual(result["reason"], reason)

    def test_unique_three_frame_acquisition_without_any_depth_fields(self):
        for count in (1, 2):
            result = self.step()
            self.assert_rejected(result, "acquiring")
            self.assertEqual(result["consecutive_valid_frames"], count)
        result = self.step()
        self.assertTrue(result["valid"])
        self.assertEqual(result["tracking_mode"], "pixel")
        np.testing.assert_allclose(result["target_m"], [.2, 0., .3])

    def test_above_moves_up_below_moves_down_without_changing_x(self):
        for pixel_y, expected_z in ((189.5, .305), (289.5, .295)):
            with self.subTest(pixel_y=pixel_y):
                self.tracker = tracking.ArmTracker(self.config, mode="pixel")
                result = self.acquire((319.5, pixel_y))
                np.testing.assert_allclose(result["target_m"], [.2, 0., expected_z])
                self.assertEqual(result["pixel_error_y"], 239.5 - pixel_y)

    def test_left_moves_chassis_positive_y_and_right_moves_negative_y(self):
        for pixel_x, expected_y in ((269.5, .005), (369.5, -.005)):
            with self.subTest(pixel_x=pixel_x):
                self.tracker = tracking.ArmTracker(self.config, mode="pixel")
                result = self.acquire((pixel_x, 239.5))
                np.testing.assert_allclose(result["target_m"], [.2, expected_y, .3])
                self.assertEqual(result["pixel_reference_x"], 319.5)
                self.assertEqual(result["pixel_error_x"], 319.5 - pixel_x)
                self.assertAlmostEqual(result["y_correction_m"], expected_y)

    def test_horizontal_deadband_minimum_correction_and_clamp(self):
        for error, correction in ((0., 0.), (8., 0.), (-8., 0.), (9., .003),
                                  (-9., -.003), (50., .005), (300., .010),
                                  (-300., -.010)):
            with self.subTest(error=error):
                self.tracker = tracking.ArmTracker(self.config, mode="pixel")
                result = self.acquire((319.5 - error, 239.5))
                self.assertAlmostEqual(result["y_correction_m"], correction)
                np.testing.assert_allclose(result["target_m"], [.2, correction, .3])

    def test_horizontal_and_vertical_deadbands_are_independent(self):
        for dx, dy, expected in ((8., -8., [.2, 0., .3]),
                                 (50., 8., [.2, .005, .3]),
                                 (-8., 50., [.2, 0., .305]),
                                 (-50., -50., [.2, -.005, .295])):
            with self.subTest(dx=dx, dy=dy):
                self.tracker = tracking.ArmTracker(self.config, mode="pixel")
                result = self.acquire((319.5 - dx, 239.5 - dy))
                np.testing.assert_allclose(result["target_m"], expected)

    def test_pixel_deadband_minimum_correction_and_clamp(self):
        for error, correction in ((0., 0.), (8., 0.), (-8., 0.), (9., .003),
                                  (-9., -.003), (50., .005), (200., .010),
                                  (-200., -.010)):
            with self.subTest(error=error):
                self.tracker = tracking.ArmTracker(self.config, mode="pixel")
                result = self.acquire((319.5, 239.5 - error))
                self.assertAlmostEqual(result["z_correction_m"], correction)
                self.assertAlmostEqual(result["target_m"][2], .3 + correction)

    def test_depth_invalidity_does_not_affect_pixel_tracking(self):
        for _ in range(3):
            result = self.step(record={"valid": True, "device_timestamp_ms": self.tick * 30.,
                                      "midpoint_px": [320., 150.], "image_size": [640, 480],
                                      "position_3d": {"valid": False,
                                                      "status": "insufficient_depth_samples"}})
        self.assertTrue(result["valid"])

    def test_lost_markers_clear_target_and_require_three_new_frames(self):
        self.acquire()
        self.assert_rejected(self.step(record={"valid": False,
                             "device_timestamp_ms": 120.}), "invalid_detection")
        self.assert_rejected(self.step(), "acquiring")
        self.assert_rejected(self.step(), "acquiring")
        self.assertTrue(self.step()["valid"])

    def test_pixel_jump_is_detected_even_when_commands_saturate(self):
        self.acquire((319.5, 100.))
        result = self.step((319.5, 10.))
        self.assert_rejected(result, "target_jump")
        self.assertEqual(result["z_correction_m"], .010)
        self.assert_rejected(self.step((319.5, 10.)), "acquiring")
        self.assertTrue(self.step((319.5, 10.))["valid"])

    def test_horizontal_jump_requires_reacquisition_even_at_command_limit(self):
        self.acquire((150., 239.5))
        result = self.step((50., 239.5))
        self.assert_rejected(result, "target_jump")
        self.assertEqual(result["y_correction_m"], .010)
        self.assert_rejected(self.step((50., 239.5)), "acquiring")
        recovered = self.step((50., 239.5))
        self.assertTrue(recovered["valid"])
        np.testing.assert_allclose(recovered["target_m"], [.2, .010, .3])

    def test_feedback_jump_and_image_size_change_require_reacquisition(self):
        self.acquire()
        self.assert_rejected(self.step(arm_pose=[.2, .4, 0.]), "target_jump")
        self.assert_rejected(self.step(record={"valid": True,
                             "device_timestamp_ms": 150., "midpoint_px": [319.5, 239.5],
                             "image_size": [1280, 960]}, arm_pose=[.2, .4, 0.]), "target_jump")

    def test_bad_pixel_values_and_image_dimensions_are_rejected(self):
        for midpoint in (None, [1.], [1., 2., 3.], [math.nan, 10.], [1., math.inf],
                         [-1., 1.], [1., -1.], [640., 2.], [1., 480.], [True, False],
                         ["320", "240"]):
            with self.subTest(midpoint=midpoint):
                self.assert_rejected(self.step(midpoint), "invalid_geometry")
        for size in (None, [640], [0, 480], [640, 1], [640.5, 480], [math.inf, 480]):
            with self.subTest(size=size):
                self.assert_rejected(self.step(record={"valid": True,
                                     "device_timestamp_ms": (self.tick + 1) * 30.,
                                     "midpoint_px": [20, 20], "image_size": size}),
                                     "invalid_geometry")

    def test_fresh_and_unique_frame_checks_still_apply(self):
        self.acquire()
        self.assert_rejected(self.step(record={"valid": True,
                             "device_timestamp_ms": 90.}), "duplicate_or_out_of_order_frame")
        self.assert_rejected(self.step(frame_received_at=1.), "stale_frame")
        self.assert_rejected(self.step(frame_received_at=100.), "future_frame")
        self.assert_rejected(self.step(frame_received_at=math.nan), "invalid_time")

    def test_frame_gap_requires_reacquisition(self):
        self.acquire()
        self.tick += 30
        self.assert_rejected(self.step(), "acquiring")
        self.assert_rejected(self.step(), "acquiring")
        self.assertTrue(self.step()["valid"])

    def test_missing_stale_or_invalid_feedback_cannot_generate_a_target(self):
        self.assert_rejected(self.step(arm_pose=None), "missing_arm_pose")
        self.assert_rejected(self.step(pose_received_at=None), "missing_arm_pose")
        self.assert_rejected(self.step(pose_received_at=1.), "stale_arm_pose")
        self.assert_rejected(self.step(pose_received_at=100.), "invalid_pose_time")
        self.assert_rejected(self.step(arm_pose=[.2, math.nan, 0.]), "invalid_geometry")

    def test_large_pitch_is_rejected_before_direction_can_reverse(self):
        for pitch in (math.radians(31), -math.radians(31), math.pi / 2., math.pi):
            with self.subTest(pitch=pitch):
                self.assert_rejected(self.step(arm_pose=[.2, .3, pitch]),
                                     "pixel_camera_not_upright")
        for pitch in (math.radians(-20), 0., math.radians(20)):
            self.tracker = tracking.ArmTracker(self.config, mode="pixel")
            self.assertGreater(self.acquire((319.5, 189.5),
                               arm_pose=[.2, .3, pitch])["target_m"][2], .3)

    def test_pixel_mode_requires_the_confirmed_tool_camera_orientation(self):
        for cfg in (config(), config("tool")):
            with self.assertRaisesRegex(ValueError, "forward-facing tool camera"):
                tracking.ArmTracker(cfg, mode="pixel")
        with self.assertRaisesRegex(ValueError, "tracking mode"):
            tracking.ArmTracker(self.config, mode="typo")


class PixelDepthTrackerTests(unittest.TestCase):
    def setUp(self):
        self.config = tracking.load_config(ROOT / "src/config/arm_tracking.json")
        self.tracker = tracking.ArmTracker(self.config, mode="pixel-depth")
        self.tick = 0

    def step(self, depth=1., midpoint=(319.5, 239.5), **kwargs):
        self.tick += 1
        now = 10. + self.tick * .03
        record = kwargs.pop("record", detection(self.tick * 30., (0., 0., depth)))
        record.update(midpoint_px=midpoint, image_size=[640, 480])
        return self.tracker.evaluate(
            record, kwargs.pop("now_monotonic", now), kwargs.pop("frame_received_at", now - .01),
            kwargs.pop("arm_pose", [.2, .3, 0.]), kwargs.pop("pose_received_at", now - .01))

    def acquire(self, depth=1., **kwargs):
        for _ in range(3):
            result = self.step(depth, **kwargs)
        self.assertTrue(result["valid"], result)
        return result

    def assert_rejected(self, result, reason):
        self.assertFalse(result["valid"])
        self.assertIsNone(result["target_m"])
        self.assertEqual(result["reason"], reason)

    def test_reference_requires_three_stable_frames_and_starts_without_x_motion(self):
        for count, depth in enumerate((.996, .998), 1):
            result = self.step(depth)
            self.assert_rejected(result, "acquiring")
            self.assertEqual(result["consecutive_valid_frames"], count)
            self.assertIsNone(result["depth_reference_m"])
        result = self.step(1.)
        self.assertTrue(result["valid"])
        self.assertEqual(result["tracking_mode"], "pixel-depth")
        self.assertEqual(result["depth_reference_m"], 1.)
        self.assertEqual(result["x_correction_m"], 0.)
        np.testing.assert_allclose(result["target_m"], [.2, 0., .3])

    def test_initial_depth_jump_restarts_reference_acquisition(self):
        self.step(.6)
        self.step(.6)
        jumped = self.step(1.)
        self.assert_rejected(jumped, "target_jump")
        self.assertIsNone(jumped["depth_reference_m"])
        self.assert_rejected(self.step(1.), "acquiring")
        result = self.step(1.)
        self.assertTrue(result["valid"])
        self.assertEqual(result["depth_reference_m"], 1.)
        self.assertEqual(result["target_m"][0], .2)

    def test_far_near_depth_direction_deadband_and_correction_limit(self):
        for error, expected in ((0., 0.), (.004, 0.), (-.004, 0.),
                                (.005, 0.), (-.005, 0.), (.006, .006),
                                (-.006, -.006), (.02, .01), (-.02, -.01)):
            with self.subTest(error=error):
                self.tracker = tracking.ArmTracker(self.config, mode="pixel-depth")
                self.acquire()
                result = self.step(1. + error)
                self.assertTrue(result["valid"], result)
                self.assertAlmostEqual(result["depth_error_m"], error)
                self.assertAlmostEqual(result["x_correction_m"], expected)
                np.testing.assert_allclose(result["target_m"], [.2 + expected, 0., .3])

    def test_depth_control_preserves_pixel_y_and_z_directions(self):
        for midpoint, expected in (((269.5, 189.5), [.208, .005, .305]),
                                    ((369.5, 289.5), [.208, -.005, .295])):
            with self.subTest(midpoint=midpoint):
                self.tracker = tracking.ArmTracker(self.config, mode="pixel-depth")
                self.acquire(midpoint=midpoint)
                result = self.step(1.008, midpoint)
                self.assertTrue(result["valid"])
                np.testing.assert_allclose(result["target_m"], expected)

    def test_depth_error_is_compensated_for_camera_forward_projection(self):
        pose = [.2, .3, math.radians(20.)]
        self.acquire(arm_pose=pose)
        result = self.step(1.006, arm_pose=pose)
        self.assertTrue(result["valid"])
        expected = .006 / math.cos(pose[2])
        self.assertAlmostEqual(result["x_correction_m"], expected)
        np.testing.assert_allclose(result["target_m"], [.2 + expected, 0., .3])

    def test_raw_depth_jump_requires_reacquisition_even_when_x_is_saturated(self):
        self.acquire()
        self.assertAlmostEqual(self.step(1.02)["x_correction_m"], .01)
        result = self.step(1.2)
        self.assert_rejected(result, "target_jump")
        self.assertAlmostEqual(result["x_correction_m"], .01)
        self.assert_rejected(self.step(1.2), "acquiring")
        recovered = self.step(1.2)
        self.assertTrue(recovered["valid"])
        self.assertEqual(recovered["depth_reference_m"], 1.)
        np.testing.assert_allclose(recovered["target_m"], [.21, 0., .3])

    def test_marker_depth_and_pose_loss_preserve_the_session_reference(self):
        for failure in ("marker", "depth", "missing_pose", "stale_pose", "explicit_invalidation"):
            with self.subTest(failure=failure):
                self.tracker = tracking.ArmTracker(self.config, mode="pixel-depth")
                self.acquire()
                if failure in ("marker", "depth"):
                    record = detection((self.tick + 1) * 30.)
                    if failure == "marker":
                        record["valid"] = False
                    else:
                        record["position_3d"]["valid"] = False
                    rejected = self.step(record=record)
                elif failure == "missing_pose":
                    rejected = self.step(arm_pose=None)
                elif failure == "stale_pose":
                    rejected = self.step(pose_received_at=1.)
                else:
                    rejected = self.tracker.invalidate("arm_feedback_missing_or_stale")
                self.assertFalse(rejected["valid"])
                self.assertIsNone(rejected["target_m"])
                self.assert_rejected(self.step(1.03), "acquiring")
                self.assert_rejected(self.step(1.03), "acquiring")
                recovered = self.step(1.03)
                self.assertTrue(recovered["valid"])
                self.assertEqual(recovered["depth_reference_m"], 1.)
                np.testing.assert_allclose(recovered["target_m"], [.21, 0., .3])

    def test_frame_gap_reacquires_without_changing_reference(self):
        self.acquire()
        self.tick += 30
        self.assert_rejected(self.step(.97), "acquiring")
        self.assert_rejected(self.step(.97), "acquiring")
        recovered = self.step(.97)
        self.assertTrue(recovered["valid"])
        self.assertEqual(recovered["depth_reference_m"], 1.)
        np.testing.assert_allclose(recovered["target_m"], [.19, 0., .3])

    def test_new_tracker_acquires_a_new_session_reference(self):
        self.acquire()
        self.tracker = tracking.ArmTracker(self.config, mode="pixel-depth")
        result = self.acquire(1.2)
        self.assertEqual(result["depth_reference_m"], 1.2)
        self.assertEqual(result["x_correction_m"], 0.)
        np.testing.assert_allclose(result["target_m"], [.2, 0., .3])


if __name__ == "__main__":
    unittest.main()
