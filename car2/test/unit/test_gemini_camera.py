#!/usr/bin/env python3
"""Hardware-free Gemini acquisition and geometry regression tests."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest import mock

import cv2
import numpy as np


MODULE = Path(__file__).resolve().parents[2] / "src/thirdparty/vision/gemini_camera.py"
SPEC = importlib.util.spec_from_file_location("gemini_camera", MODULE)
gemini = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gemini)


class Profile:
    def __init__(self, width=4, height=3, fmt="BGR", fps=30):
        self.width, self.height, self.fmt, self.fps = width, height, fmt, fps
        self.intrinsic = SimpleNamespace(width=width, height=height, fx=100., fy=100.,
                                         cx=width / 2, cy=height / 2)
        self.distortion = SimpleNamespace(**{key: 0. for key in
                                             ("k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2")})

    def get_width(self): return self.width
    def get_height(self): return self.height
    def get_format(self): return self.fmt
    def get_fps(self): return self.fps
    def as_video_stream_profile(self): return self
    def get_intrinsic(self): return self.intrinsic
    def get_distortion(self): return self.distortion


class Frame(Profile):
    def __init__(self, data, width=4, height=3, fmt="BGR", scale=1.):
        super().__init__(width, height, fmt)
        self.data, self.scale = data, scale
        self.timestamp = 123.5

    def get_data(self): return self.data
    def get_depth_scale(self): return self.scale
    def get_timestamp(self): return self.timestamp
    def get_stream_profile(self): return self


class Frames:
    def __init__(self, color, depth):
        self.color, self.depth = color, depth

    def get_color_frame(self): return self.color
    def get_depth_frame(self): return self.depth
    def as_frame_set(self): return self


class FakeSDK:
    OBFormat = SimpleNamespace(BGR="BGR", RGB="RGB", MJPG="MJPG", Y16="Y16")
    OBSensorType = SimpleNamespace(COLOR_SENSOR="color", DEPTH_SENSOR="depth")
    OBFrameAggregateOutputMode = SimpleNamespace(COLOR_FRAME_REQUIRE="color_required")
    OBStreamType = SimpleNamespace(COLOR_STREAM="color")
    OBPropertyID = SimpleNamespace(OB_PROP_DISPARITY_TO_DEPTH_BOOL="disparity_to_depth")

    def __init__(self, names=("Gemini 335",)):
        self.disparity_enabled = True
        self.property_error = False
        def get_bool_property(key):
            if self.property_error:
                raise RuntimeError("property unavailable")
            return self.disparity_enabled
        self.devices = []
        for index, name in enumerate(names):
            info = SimpleNamespace(get_name=lambda n=name: n,
                                   get_serial_number=lambda i=index: f"serial-{i}")
            self.devices.append(SimpleNamespace(get_device_info=lambda info=info: info,
                                                get_bool_property=get_bool_property))
        self.context_calls = 0
        self.start_calls = self.stop_calls = self.sync_calls = self.align_calls = 0
        self.streams = []
        self.mode = self.align_target = None
        self.target_distortion = 0.
        self.reject_target_distortion = False
        self.start_error = self.stop_error = False
        self.supported_colors = {"BGR", "RGB", "MJPG"}
        self.unsupported_depth_profiles = set()
        self.color = Frame(np.full((3, 4, 3), [10, 20, 30], dtype=np.uint8))
        self.depth = Frame(np.full((3, 4), 1000, dtype=np.uint16), fmt="Y16")
        self.frames = Frames(self.color, self.depth)
        self.aligned = self.frames
        self.profile_requests = []

    def Context(self):
        self.context_calls += 1
        devices = SimpleNamespace(get_count=lambda: len(self.devices),
                                  get_device_by_index=lambda i: self.devices[i])
        return SimpleNamespace(query_devices=lambda: devices)

    def Pipeline(self, device):
        return self

    def Config(self):
        return SimpleNamespace(enable_stream=self.streams.append,
                               set_frame_aggregate_output_mode=lambda value: setattr(self, "mode", value))

    def get_stream_profile_list(self, sensor):
        def get(width, height, fmt, fps):
            self.profile_requests.append((sensor, width, height, fmt, fps))
            if sensor == "color" and fmt not in self.supported_colors:
                raise RuntimeError("unsupported")
            if sensor == "depth" and (width, height, fmt, fps) in self.unsupported_depth_profiles:
                raise RuntimeError("unsupported depth profile")
            return Profile(width or 8, height or 6, fmt, fps)
        return SimpleNamespace(get_video_stream_profile=get)

    def enable_frame_sync(self):
        self.sync_calls += 1

    def AlignFilter(self, target):
        self.align_target = target
        def process(frames):
            self.align_calls += 1
            return self.aligned
        def set_config_value(name, value):
            if not self.reject_target_distortion:
                self.target_distortion = value
        return SimpleNamespace(process=process, set_config_value=set_config_value,
                               get_config_value=lambda name: self.target_distortion)

    def start(self, config):
        self.start_calls += 1
        if self.start_error:
            raise RuntimeError("start failed")

    def stop(self):
        self.stop_calls += 1
        if self.stop_error:
            raise RuntimeError("stop failed")

    def wait_for_frames(self, timeout):
        self.timeout = timeout
        return self.frames


class CameraTests(unittest.TestCase):
    def camera(self, **kwargs):
        sdk = kwargs.pop("sdk", FakeSDK())
        camera = gemini.GeminiCamera(width=4, height=3, sdk=sdk, **kwargs)
        camera.start()
        self.addCleanup(camera.stop)
        return camera, sdk

    def test_construct_does_not_open_hardware_and_missing_sdk_is_clear(self):
        sdk = FakeSDK()
        gemini.GeminiCamera(sdk=sdk)
        self.assertEqual(sdk.context_calls, 0)
        with mock.patch.object(gemini.importlib, "import_module", side_effect=ImportError("missing")):
            with self.assertRaisesRegex(RuntimeError, "pyorbbecsdk2"):
                gemini.GeminiCamera().start()

    def test_profiles_sync_alignment_and_units(self):
        camera, sdk = self.camera()
        sdk.depth.scale = .5
        metadata = camera.start()
        self.assertEqual(metadata["serial"], "serial-0")
        self.assertEqual(metadata["depth_profile"]["width"], 8)
        self.assertIn(("depth", 0, 0, "Y16", 30), sdk.profile_requests)
        self.assertEqual(sdk.sync_calls, 1)
        self.assertEqual(sdk.align_target, "color")
        self.assertEqual(sdk.target_distortion, 1.)
        self.assertEqual(sdk.mode, "color_required")
        self.assertEqual(sdk.start_calls, 1)
        result = camera.read()
        np.testing.assert_array_equal(result["color_bgr"][0, 0], [10, 20, 30])
        np.testing.assert_allclose(result["depth_m"], .5)
        self.assertEqual(result["timestamp_ms"], 123.5)
        self.assertEqual(result["distortion"].shape, (8,))
        self.assertEqual(result["depth_status"], "available")
        sdk.color.data[:] = 0
        np.testing.assert_array_equal(result["color_bgr"][0, 0], [10, 20, 30])

    def test_explicit_depth_dimensions_are_independent_of_color_dimensions(self):
        camera, sdk = self.camera(depth_width=640, depth_height=360)
        metadata = camera.start()
        self.assertEqual(sdk.profile_requests,
                         [("color", 4, 3, "BGR", 30),
                          ("depth", 640, 360, "Y16", 30)])
        self.assertEqual(metadata["color_profile"],
                         {"width": 4, "height": 3, "fps": 30, "format": "BGR"})
        self.assertEqual(metadata["depth_profile"],
                         {"width": 640, "height": 360, "fps": 30, "format": "Y16"})
        self.assertEqual([(profile.width, profile.height) for profile in sdk.streams],
                         [(4, 3), (640, 360)])

    def test_unavailable_explicit_depth_profile_fails_without_automatic_fallback(self):
        sdk = FakeSDK()
        sdk.unsupported_depth_profiles.add((640, 360, "Y16", 30))
        camera = gemini.GeminiCamera(depth_width=640, depth_height=360, sdk=sdk)
        self.addCleanup(camera.stop)
        with self.assertRaisesRegex(RuntimeError, r"depth.*640x360"):
            camera.start()
        depth_requests = [request for request in sdk.profile_requests if request[0] == "depth"]
        self.assertEqual(depth_requests, [("depth", 640, 360, "Y16", 30)])
        self.assertEqual(sdk.start_calls, 0)
        self.assertEqual(sdk.sync_calls, 0)
        self.assertIsNone(camera._pipeline)

    def test_rgb_profile_fallback_and_conversion(self):
        sdk = FakeSDK()
        sdk.supported_colors = {"RGB"}
        sdk.color.fmt = "RGB"
        camera, _ = self.camera(sdk=sdk)
        np.testing.assert_array_equal(camera.read()["color_bgr"][0, 0], [30, 20, 10])
        self.assertEqual(sdk.streams[0].fmt, "RGB")

    def test_mjpg_decoding_and_bad_dimensions(self):
        camera, sdk = self.camera()
        ok, data = cv2.imencode(".jpg", np.full((3, 4, 3), 180, dtype=np.uint8))
        self.assertTrue(ok)
        sdk.color.fmt, sdk.color.data = "MJPG", data
        self.assertEqual(camera.read()["color_bgr"].shape, (3, 4, 3))
        sdk.color.width = 5
        with self.assertRaisesRegex(ValueError, "MJPG"):
            camera.read()

    def test_bad_color_buffer_and_unsupported_format(self):
        camera, sdk = self.camera()
        sdk.color.data = b"short"
        with self.assertRaisesRegex(ValueError, "buffer length"):
            camera.read()
        sdk.color.fmt = "unknown"
        with self.assertRaisesRegex(ValueError, "unsupported"):
            camera.read()

    def test_color_only_never_requests_or_aligns_depth(self):
        camera, sdk = self.camera(color_only=True)
        self.assertEqual(len(sdk.streams), 1)
        self.assertEqual(sdk.sync_calls, 0)
        self.assertEqual(camera.read()["depth_status"], "disabled")
        self.assertIsNone(camera.read()["depth_m"])
        self.assertEqual(sdk.align_calls, 0)

    def test_color_only_does_not_query_configured_depth_profile(self):
        sdk = FakeSDK()
        sdk.unsupported_depth_profiles.add((640, 360, "Y16", 30))
        with mock.patch.object(sdk, "get_stream_profile_list",
                               wraps=sdk.get_stream_profile_list) as profiles:
            camera, sdk = self.camera(sdk=sdk, color_only=True,
                                      depth_width=640, depth_height=360)
        self.assertEqual(profiles.call_args_list, [mock.call("color")])
        self.assertEqual(sdk.profile_requests, [("color", 4, 3, "BGR", 30)])
        self.assertIsNone(camera.start()["depth_profile"])
        self.assertEqual(camera.read()["depth_status"], "disabled")
        self.assertEqual(sdk.sync_calls, 0)
        self.assertIsNone(sdk.align_target)

    def test_no_camera_wrong_model_and_multiple_cameras(self):
        for names in ((), ("Gemini 335L",), ("Gemini 336",),
                      ("Gemini 335", "Gemini 335")):
            with self.subTest(names=names):
                sdk = FakeSDK(names)
                with self.assertRaises(RuntimeError):
                    gemini.GeminiCamera(sdk=sdk).start()
                self.assertEqual(sdk.start_calls, 0)
        camera, _ = self.camera(sdk=FakeSDK(("Gemini 335", "Gemini 335")),
                                serial="serial-1")
        self.assertEqual(camera.start()["serial"], "serial-1")

    def test_start_failure_cleanup_and_idempotent_stop(self):
        sdk = FakeSDK()
        sdk.start_error = True
        camera = gemini.GeminiCamera(sdk=sdk)
        with self.assertRaisesRegex(RuntimeError, "start failed"):
            camera.start()
        self.assertEqual(sdk.stop_calls, 1)
        camera.stop()
        self.assertEqual(sdk.stop_calls, 1)
        sdk.start_error = False
        camera.start()
        camera.stop()
        camera.stop()
        self.assertEqual(sdk.stop_calls, 2)

    def test_start_failure_is_preserved_when_cleanup_fails(self):
        sdk = FakeSDK()
        sdk.start_error = sdk.stop_error = True
        camera = gemini.GeminiCamera(sdk=sdk)
        with self.assertRaisesRegex(RuntimeError, "start failed"):
            camera.start()
        self.assertIsNone(camera._pipeline)

    def test_missing_frames_and_color_raise_timeout(self):
        camera, sdk = self.camera()
        sdk.frames = None
        with self.assertRaises(TimeoutError):
            camera.read()
        sdk.frames = Frames(None, sdk.depth)
        with self.assertRaises(TimeoutError):
            camera.read()

    def test_depth_problems_preserve_color(self):
        for kind, status in (("missing", "missing_depth"), ("align_none", "alignment_failed"),
                             ("align_generic_none", "alignment_failed"),
                             ("wrong_size", "alignment_size_mismatch"),
                             ("wrong_format", "invalid_depth"),
                             ("short_buffer", "invalid_depth"), ("bad_scale", "invalid_depth"),
                             ("bad_calibration", "invalid_calibration"),
                             ("bad_calibration_size", "invalid_calibration"),
                             ("zero", "no_valid_depth")):
            with self.subTest(kind=kind):
                camera, sdk = self.camera()
                if kind == "missing": sdk.frames.depth = None
                elif kind == "align_none": sdk.aligned = None
                elif kind == "align_generic_none":
                    sdk.aligned = SimpleNamespace(as_frame_set=lambda: None)
                elif kind == "wrong_size": sdk.depth.width = 5
                elif kind == "wrong_format": sdk.depth.fmt = "RGB"
                elif kind == "short_buffer": sdk.depth.data = b"short"
                elif kind == "bad_scale": sdk.depth.scale = float("nan")
                elif kind == "bad_calibration": sdk.color.intrinsic.fx = float("nan")
                elif kind == "bad_calibration_size": sdk.color.intrinsic.width = 5
                elif kind == "zero": sdk.depth.data[:] = 0
                result = camera.read()
                self.assertTrue(result["depth_status"].startswith(status))
                self.assertEqual(result["color_bgr"].shape, (3, 4, 3))
                if kind != "zero":
                    self.assertIsNone(result["depth_m"])

    def test_target_distortion_must_be_enabled(self):
        sdk = FakeSDK()
        sdk.reject_target_distortion = True
        with self.assertRaisesRegex(RuntimeError, "distorted color"):
            gemini.GeminiCamera(sdk=sdk).start()
        self.assertEqual(sdk.start_calls, 0)

    def test_unconverted_or_unverified_disparity_does_not_become_depth(self):
        for property_error in (False, True):
            with self.subTest(property_error=property_error):
                sdk = FakeSDK()
                sdk.disparity_enabled = False
                sdk.property_error = property_error
                camera, sdk = self.camera(sdk=sdk)
                result = camera.read()
                self.assertIsNone(result["depth_m"])
                self.assertEqual(result["color_bgr"].shape, (3, 4, 3))
                self.assertTrue(result["depth_status"].startswith("disparity_conversion_"))
                self.assertEqual(sdk.align_calls, 0)

    def test_depth_timestamp_mismatch_is_rejected(self):
        camera, sdk = self.camera()
        sdk.depth.timestamp += 1000. / 30
        result = camera.read()
        self.assertEqual(result["depth_status"], "depth_timestamp_mismatch")
        self.assertIsNone(result["depth_m"])
        self.assertEqual(sdk.align_calls, 0)

    def test_aligned_depth_calibration_must_match_raw_rgb(self):
        for changed in ("intrinsic", "distortion"):
            with self.subTest(changed=changed):
                camera, sdk = self.camera()
                if changed == "intrinsic":
                    sdk.depth.intrinsic.cx += 1.
                else:
                    sdk.depth.distortion.k1 = .2
                result = camera.read()
                self.assertEqual(result["depth_status"], "alignment_calibration_mismatch")
                self.assertIsNone(result["depth_m"])

    def test_invalid_options(self):
        for kwargs in ({"width": 0}, {"height": -1}, {"fps": True}, {"timeout_ms": 1.5}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                gemini.GeminiCamera(**kwargs)

    def test_depth_dimensions_must_be_nonnegative_integers_and_set_as_pair(self):
        for width, height in ((-1, 360), (640, -1), (1.5, 360), (640, 360.),
                              ("640", 360), (640, "360"), (True, 1), (1, False),
                              (None, 360), (640, None), (0, 360), (640, 0)):
            with self.subTest(width=width, height=height):
                sdk = FakeSDK()
                with self.assertRaises(ValueError):
                    gemini.GeminiCamera(depth_width=width, depth_height=height, sdk=sdk)
                self.assertEqual(sdk.context_calls, 0)


def geometry_case():
    codes = []
    for x, y in ((20., 40.), (80., 40.)):
        codes.append({"center_px": [x, y],
                      "corners_px": [[x - 8, y - 8], [x + 8, y - 8],
                                     [x + 8, y + 8], [x - 8, y + 8]]})
    detection = {"valid": True, "codes": codes, "pair_indices": [0, 1]}
    depth = np.full((80, 100), 8., dtype=float)  # Background must never be used.
    depth[32:49, 12:29] = 1.
    depth[32:49, 72:89] = 2.
    frame = {"color_bgr": np.zeros((80, 100, 3), dtype=np.uint8),
             "depth_m": depth, "depth_status": "available",
             "camera_matrix": np.array([[100., 0., 50.], [0., 100., 40.], [0., 0., 1.]]),
             "distortion": np.zeros(8)}
    return detection, frame


class GeometryTests(unittest.TestCase):
    def test_true_3d_midpoint_averages_two_centers(self):
        detection, frame = geometry_case()
        result = gemini.locate_midpoint(detection, frame)
        self.assertTrue(result["valid"])
        np.testing.assert_allclose(result["centers_m"], [[-.3, 0., 1.], [.6, 0., 2.]])
        np.testing.assert_allclose(result["midpoint_m"], [.15, 0., 1.5])
        self.assertEqual(result["frame_id"], "gemini_color_optical_frame")

    def test_depth_filters_zero_nan_and_infinity(self):
        detection, frame = geometry_case()
        frame["depth_m"][38:43, 18:23] = np.nan
        frame["depth_m"][40, 18:23] = [0., 1., 1., 1., np.inf]
        result = gemini.locate_midpoint(detection, frame)
        self.assertTrue(result["valid"])
        np.testing.assert_allclose(result["midpoint_m"], [.15, 0., 1.5])
        frame["depth_m"][40, 21] = 0.
        self.assertEqual(gemini.locate_midpoint(detection, frame)["status"],
                         "insufficient_depth_samples")

    def test_reject_depth_discontinuity(self):
        detection, frame = geometry_case()
        frame["depth_m"][40, 20] = 5.
        result = gemini.locate_midpoint(detection, frame)
        self.assertFalse(result["valid"])
        self.assertEqual(result["status"], "depth_discontinuity")
        self.assertIsNone(result["midpoint_m"])

    def test_samples_stay_inside_quad(self):
        detection, frame = geometry_case()
        detection["codes"][0]["corners_px"] = [[18., 38.], [22., 38.], [22., 42.], [18., 42.]]
        frame["depth_m"][32:49, 12:29] = 9.
        frame["depth_m"][38:43, 18:23] = 1.
        result = gemini.locate_midpoint(detection, frame, sample_radius=4)
        self.assertTrue(result["valid"])
        self.assertEqual(result["centers_m"][0][2], 1.)

    def test_tiny_quad_and_absent_depth_rejected(self):
        detection, frame = geometry_case()
        detection["codes"][0]["corners_px"] = [[19.9, 39.9], [20.1, 39.9],
                                               [20.1, 40.1], [19.9, 40.1]]
        self.assertFalse(gemini.locate_midpoint(detection, frame)["valid"])
        detection, frame = geometry_case()
        frame["depth_m"] = None
        frame["depth_status"] = "missing_depth"
        self.assertEqual(gemini.locate_midpoint(detection, frame)["status"], "missing_depth")

    def test_undistortion_uses_actual_color_calibration(self):
        detection, frame = geometry_case()
        frame["distortion"] = np.array([.15, -.02, .003, -.004, .01, .002, 0., 0.])
        expected_rays = np.array([[-.3, 0.], [.3, 0.]])
        projected, _ = cv2.projectPoints(
            np.column_stack((expected_rays, np.ones(2))), np.zeros(3), np.zeros(3),
            frame["camera_matrix"], frame["distortion"])
        for code, point in zip(detection["codes"], projected.reshape(2, 2)):
            code["center_px"] = point.tolist()
        result = gemini.locate_midpoint(detection, frame)
        self.assertTrue(result["valid"])
        np.testing.assert_allclose(result["midpoint_m"], [.15, 0., 1.5], atol=1e-7)

    def test_no_stale_result_and_reordered_pair(self):
        detection, frame = geometry_case()
        detection["pair_indices"] = [1, 0]
        self.assertEqual(gemini.locate_midpoint(detection, frame)["centers_m"][0][2], 2.)
        detection["valid"] = False
        result = gemini.locate_midpoint(detection, frame)
        self.assertFalse(result["valid"])
        self.assertIsNone(result["centers_m"])

    def test_nonconvergent_distortion_rejected(self):
        detection, frame = geometry_case()
        frame["distortion"][0] = -10.
        result = gemini.locate_midpoint(detection, frame)
        self.assertFalse(result["valid"])
        self.assertEqual(result["status"], "invalid_calibration")
        self.assertIsNone(result["midpoint_m"])

    def test_bad_inputs_and_calibration_rejected(self):
        for change, status in (
                (lambda d, f: d.update(pair_indices=[0, 0]), "invalid_detection"),
                (lambda d, f: d.update(pair_indices=[-1, 0]), "invalid_detection"),
                (lambda d, f: d["codes"][0].update(center_px=[float("nan"), 1.]), "invalid_detection"),
                (lambda d, f: f.update(camera_matrix=None), "invalid_calibration"),
                (lambda d, f: f.update(distortion=np.zeros(5)), "invalid_calibration"),
                (lambda d, f: f.update(depth_m=np.zeros((2, 2))), "alignment_size_mismatch")):
            with self.subTest(status=status):
                detection, frame = geometry_case()
                change(detection, frame)
                result = gemini.locate_midpoint(detection, frame)
                self.assertFalse(result["valid"])
                self.assertEqual(result["status"], status)
        with self.assertRaises(ValueError):
            gemini.locate_midpoint(*geometry_case(), sample_radius=-1)


if __name__ == "__main__":
    unittest.main()
