"""Tracker lifecycle tests use generated markers and fake ROS/camera objects only."""

import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ENTRY = ROOT / "src/example/arm_tracking.py"
spec = importlib.util.spec_from_file_location("arm_tracking_entry_test", ENTRY)
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)


def pair_image():
    image = np.full((300, 600, 3), 255, dtype=np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for x, marker_id in ((40, 0), (340, 1)):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 160)
        image[60:220, x:x + 160] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return image


class TrackingEntrypointTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="arm tracking test ")
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.input = self.directory / "pair.png"
        cv2.imwrite(str(self.input), pair_image())
        self.config = self.directory / "calibration.json"
        data = json.loads((ROOT / "src/config/arm_tracking.json").read_text())
        data.update(calibrated=True, camera_mount="tool",
                    rotation=[[0, 0, 1], [-1, 0, 0], [0, -1, 0]],
                    translation_m=[0, 0, 0])
        self.config.write_text(json.dumps(data))
        self.uncalibrated = self.directory / "uncalibrated.json"
        data.update(calibrated=False, rotation=None, translation_m=None)
        self.uncalibrated.write_text(json.dumps(data))

    def launch(self, *args):
        return subprocess.run([sys.executable, "-B", str(ENTRY), *map(str, args)],
                              capture_output=True, text=True, cwd=self.directory, timeout=10)

    def test_help_does_not_require_opencv_ros_or_camera(self):
        result = subprocess.run([sys.executable, "-B", "-S", str(ENTRY), "--help"],
                                capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--execute", result.stdout)
        self.assertIn("--joystick", result.stdout)
        self.assertIn("--tracking-mode", result.stdout)
        self.assertIn("button6", result.stdout)
        self.assertIn("chassis left/right", result.stdout)
        self.assertNotIn("button2", result.stdout)

    def test_default_offline_preview_never_enables_control(self):
        result = self.launch("--image", self.input, "--headless")
        self.assertEqual(result.returncode, 0, result.stderr)
        records = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertTrue(records[0]["valid"])
        self.assertFalse(records[0]["tracking"]["valid"])
        self.assertEqual(records[0]["tracking"]["mode"], "preview")
        self.assertEqual(records[0]["tracking"]["tracking_mode"], "pixel")
        self.assertFalse(records[-1]["tracking"]["valid"])

    def test_execute_requires_calibration_before_device_access(self):
        with mock.patch("vision.gemini_camera.GeminiCamera") as camera, \
                mock.patch("vision.arm_tracking_ros.TrackingRosLink") as ros, \
                contextlib.redirect_stderr(io.StringIO()) as errors:
            code = entry.main(["--execute", "--headless", "--tracking-config", str(self.uncalibrated)])
        self.assertEqual(code, 1)
        self.assertIn("not calibrated", errors.getvalue())
        camera.assert_not_called()
        ros.assert_not_called()

    def test_control_cannot_use_offline_image_and_depth_cannot_use_color_only(self):
        for mode in ("--execute", "--joystick"):
            for option in (("--image", self.input),
                           ("--tracking-mode", "depth", "--image", self.input),
                           ("--tracking-mode", "depth", "--color-only"),
                           ("--tracking-mode", "pixel-depth", "--image", self.input),
                           ("--tracking-mode", "pixel-depth", "--color-only"),
                           ("--tracking-mode", "approach", "--image", self.input),
                           ("--tracking-mode", "approach", "--color-only")):
                result = self.launch(mode, "--headless", *option)
                self.assertEqual(result.returncode, 2, result.stderr)
        result = self.launch("--execute", "--joystick", "--headless")
        self.assertEqual(result.returncode, 2)

    def test_outputs_cannot_overwrite_calibration_even_via_hardlink(self):
        alias = self.directory / "alias.json"
        os.link(self.config, alias)
        before = self.config.read_bytes()
        for path in (self.config, alias):
            result = self.launch("--image", self.input, "--headless", "--tracking-config",
                                 self.config, "--json-output", path)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertEqual(self.config.read_bytes(), before)

    @staticmethod
    def frame(index, depth=True, distance=1.):
        return dict(color_bgr=pair_image(), timestamp_ms=1000.0 + index * 33.3,
                    depth_m=np.full((300, 600), distance) if depth else None,
                    camera_matrix=np.array([[500., 0, 300], [0, 500., 150], [0, 0, 1.]]),
                    distortion=np.zeros(8), depth_status="available" if depth else "missing_depth")

    def fake_run(self, frames, *args, pose=True, requests=None, poses=None, time_step=None,
                 cancel_during_enable=False, enable_error=None,
                 reenter_during_enable=False, disable_error=None):
        camera, ros = mock.Mock(), mock.Mock()
        clock = [time.monotonic()]
        clock_context = (mock.patch.object(entry.time, "monotonic", lambda: clock[0])
                         if time_step is not None else contextlib.nullcontext())
        camera.start.return_value = {"name": "fake"}
        frame_iter = iter(frames)
        request_iter = iter(requests) if requests is not None else None
        pose_iter = iter(poses) if poses is not None else None
        pose_state = [[.14, .15, 0.] if pose else None]
        request_state = [False]
        cancelled = [False]

        def read():
            if time_step is not None:
                clock[0] += time_step
            frame = next(frame_iter)
            if isinstance(frame, BaseException):
                raise frame
            if request_iter is not None:
                requested = next(request_iter)
                cancelled[0] |= request_state[0] and not requested
                request_state[0] = requested
            if pose_iter is not None:
                pose_state[0] = next(pose_iter)
            return frame

        camera.read.side_effect = read
        ros.enabled = False
        def enable():
            if enable_error:
                raise RuntimeError(enable_error)
            ros.enabled = True
            if cancel_during_enable:
                cancelled[0] = True
                request_state[0] = reenter_during_enable

        ros.enable.side_effect = enable
        def disable():
            if disable_error:
                raise RuntimeError(disable_error)
            ros.enabled = False

        def take_cancel():
            result = cancelled[0]
            cancelled[0] = False
            return result

        ros.disable.side_effect = disable
        ros.take_joystick_cancel.side_effect = take_cancel
        ros.require_joystick_release.side_effect = lambda: request_state.__setitem__(0, False)
        ros.joystick_requested.side_effect = lambda now: request_state[0]
        ros.fresh_pose.side_effect = lambda now: (
            (pose_state[0], now) if pose_state[0] is not None else (None, None))
        events = []
        ros.close.side_effect = lambda: events.append("ros_close")
        camera.stop.side_effect = lambda: events.append("camera_stop")
        with clock_context, \
                mock.patch("vision.gemini_camera.GeminiCamera", return_value=camera) as camera_factory, \
                mock.patch("vision.arm_tracking_ros.TrackingRosLink", return_value=ros) as ros_factory, \
                contextlib.redirect_stdout(io.StringIO()) as output, \
                contextlib.redirect_stderr(io.StringIO()):
            code = entry.main(["--tracking-config", str(self.config), "--headless",
                               "--print-hz", "100000", *args])
        camera.configuration = camera_factory.call_args.kwargs
        ros.configuration = ros_factory.call_args.kwargs
        records = [json.loads(line) for line in output.getvalue().splitlines()]
        return code, records, ros, camera, events

    def test_preview_with_real_geometry_has_no_motion_side_effect(self):
        code, records, ros, camera, events = self.fake_run(
            [self.frame(i) for i in range(5)], "--max-frames", "5")
        self.assertEqual(code, 0)
        self.assertTrue(any(r["tracking"].get("valid") for r in records))
        ros.enable.assert_not_called()
        ros.publish.assert_not_called()
        ros.stop.assert_not_called()
        self.assertEqual(events, ["ros_close", "camera_stop"])

    def test_execute_waits_for_consecutive_frames_then_stops_on_lost_depth(self):
        frames = [self.frame(i) for i in range(4)] + [self.frame(4, depth=False)]
        code, records, ros, camera, events = self.fake_run(
            frames, "--execute", "--tracking-mode", "depth", "--max-frames", "5")
        self.assertEqual(code, 0)
        ros.enable.assert_called_once()
        ros.publish.assert_called_once()
        self.assertEqual(records[2]["tracking"]["reason"], "arming_wait_for_new_frame")
        self.assertFalse(records[-2]["tracking"]["valid"])
        self.assertIsNone(records[-2]["tracking"]["target_m"])
        self.assertGreaterEqual(ros.stop.call_count, 1)
        self.assertEqual(events, ["ros_close", "camera_stop"])
        self.assertFalse(camera.configuration["color_only"])
        self.assertEqual((camera.configuration["depth_width"], camera.configuration["depth_height"]), (0, 0))

    def test_pixel_control_uses_color_only_and_never_projects_depth(self):
        with mock.patch("vision.gemini_camera.locate_midpoint",
                        side_effect=AssertionError("pixel mode must not inspect depth")) as project:
            code, records, ros, camera, _ = self.fake_run(
                [self.frame(i, depth=False) for i in range(5)],
                "--execute", "--color-only", "--max-frames", "5")
        self.assertEqual(code, 0)
        self.assertTrue(camera.configuration["color_only"])
        project.assert_not_called()
        ros.enable.assert_called_once()
        self.assertEqual(ros.publish.call_count, 2)
        for record in records[3:-1]:
            self.assertTrue(record["tracking"]["valid"])
            self.assertEqual(record["tracking"]["tracking_mode"], "pixel")
            self.assertEqual(record["tracking"]["pixel_error_y"], 10.)
            self.assertEqual(record["tracking"]["pixel_error_x"], 30.)
            np.testing.assert_allclose(record["tracking"]["target_m"], [.14, .003, .153])
            self.assertEqual(record["position_3d"]["status"], "not_used_in_pixel_mode")
        for call in ros.publish.call_args_list:
            np.testing.assert_allclose(call.args[0], [.14, .003, .153])

    def test_pixel_right_target_publishes_negative_chassis_y(self):
        frames = [self.frame(i, depth=False) for i in range(5)]
        for frame in frames:
            # Translate both markers right without mirroring their encoded identities.
            translated = np.full_like(frame["color_bgr"], 255)
            translated[:, 60:] = frame["color_bgr"][:, :-60]
            frame["color_bgr"] = translated
        code, records, ros, _, _ = self.fake_run(
            frames, "--execute", "--max-frames", "5")
        self.assertEqual(code, 0)
        self.assertEqual(ros.publish.call_count, 2)
        for call in ros.publish.call_args_list:
            np.testing.assert_allclose(call.args[0], [.14, -.003, .153])
        self.assertEqual(records[-2]["tracking"]["pixel_error_x"], -30.)

    def test_pixel_button6_entry_and_exit_work_without_depth(self):
        requests = [False, True, True, True, True, False]
        code, records, ros, camera, _ = self.fake_run(
            [self.frame(i, depth=False) for i in range(len(requests))],
            "--joystick", "--max-frames", str(len(requests)), requests=requests)
        self.assertEqual(code, 0)
        self.assertTrue(camera.configuration["color_only"])
        ros.enable.assert_called_once()
        ros.publish.assert_called_once()
        ros.disable.assert_called_once()
        self.assertEqual(records[4]["tracking"]["mode"], "execute")
        self.assertEqual(records[5]["tracking"]["mode"], "standby")

    def test_pixel_loss_stops_targets_even_when_depth_is_absent(self):
        frames = [self.frame(i, depth=False) for i in range(5)]
        frames[-1]["color_bgr"][:] = 255
        code, records, ros, camera, _ = self.fake_run(
            frames, "--execute", "--max-frames", "5")
        self.assertEqual(code, 0)
        ros.enable.assert_called_once()
        ros.publish.assert_called_once()
        self.assertFalse(records[-2]["tracking"]["valid"])
        self.assertEqual(records[-2]["tracking"]["reason"], "invalid_detection")
        self.assertGreater(ros.stop.call_count, 0)

    def test_pixel_depth_button_sessions_acquire_new_distances_after_standby(self):
        requests = [False] * 3 + [True] * 5 + [False] * 3 + [True] * 5 + [False]
        distances = [.6] * 3 + [1.] * 4 + [1.02] + [1.2] * 3 + [.8] * 4 + [.78, .78]
        code, records, ros, camera, _ = self.fake_run(
            [self.frame(i, distance=depth) for i, depth in enumerate(distances)],
            "--joystick", "--tracking-mode", "pixel-depth", "--max-frames", str(len(requests)),
            requests=requests)
        self.assertEqual(code, 0)
        self.assertFalse(camera.configuration["color_only"])
        self.assertEqual(ros.configuration, {"execute": True, "joystick": True,
                                             "track_x": True, "goal_mode": False})
        self.assertEqual(ros.enable.call_count, 2)
        self.assertEqual(ros.disable.call_count, 2)
        self.assertEqual(ros.publish.call_count, 4)
        for index, reference, x in ((6, 1., .14), (7, 1., .15), (14, .8, .14), (15, .8, .13)):
            result = records[index]["tracking"]
            self.assertTrue(result["valid"], result)
            self.assertAlmostEqual(result["depth_reference_m"], reference)
            np.testing.assert_allclose(result["target_m"], [x, .003, .153])
        for index in (3, 4, 11, 12):
            self.assertEqual(records[index]["tracking"]["reason"], "acquiring")
        for index in (5, 13):
            self.assertEqual(records[index]["tracking"]["reason"], "arming_wait_for_new_frame")
        self.assertEqual(records[16]["tracking"]["mode"], "standby")

    def test_pixel_depth_loss_stops_then_resumes_with_the_original_reference(self):
        for failure in ("depth", "pose"):
            with self.subTest(failure=failure):
                frames = [self.frame(i, distance=1. if i < 4 else 1.03) for i in range(8)]
                poses = [[.14, .15, 0.]] * len(frames)
                if failure == "depth":
                    frames[4]["depth_m"] = None
                    frames[4]["depth_status"] = "missing_depth"
                else:
                    poses[4] = None
                code, records, ros, camera, _ = self.fake_run(
                    frames, "--execute", "--tracking-mode", "pixel-depth", "--max-frames", "8",
                    poses=poses)
                self.assertEqual(code, 0)
                self.assertFalse(camera.configuration["color_only"])
                self.assertTrue(ros.configuration["track_x"])
                ros.enable.assert_called_once()
                self.assertEqual(ros.publish.call_count, 2)
                self.assertFalse(records[4]["tracking"]["valid"])
                self.assertIsNone(records[4]["tracking"]["target_m"])
                for index in (5, 6):
                    self.assertEqual(records[index]["tracking"]["reason"], "acquiring")
                recovered = records[7]["tracking"]
                self.assertTrue(recovered["valid"])
                self.assertAlmostEqual(recovered["depth_reference_m"], 1.)
                np.testing.assert_allclose(recovered["target_m"], [.15, .003, .153])
                calls = [call[0] for call in ros.method_calls if call[0] in ("publish", "stop")]
                first = calls.index("publish")
                last = len(calls) - 1 - calls[::-1].index("publish")
                self.assertIn("stop", calls[first + 1:last])

    def test_missing_arm_feedback_never_enables_execute(self):
        code, records, ros, camera, events = self.fake_run(
            [self.frame(i) for i in range(5)], "--execute", "--max-frames", "5", pose=False)
        self.assertEqual(code, 0)
        ros.enable.assert_not_called()
        ros.publish.assert_not_called()
        self.assertTrue(all(not r["tracking"]["valid"] for r in records))

    def test_joystick_stays_in_standby_until_button_request(self):
        code, records, ros, camera, _ = self.fake_run(
            [self.frame(i) for i in range(6)], "--joystick", "--max-frames", "6",
            requests=[False] * 6)
        self.assertEqual(code, 0)
        ros.enable.assert_not_called()
        ros.publish.assert_not_called()
        ros.disable.assert_not_called()
        self.assertEqual(camera.read.call_count, 6)
        self.assertTrue(all(r["tracking"]["mode"] == "standby" for r in records[:-1]))
        self.assertTrue(all(r["tracking"]["reason"] == "button6_to_start"
                            for r in records[:-1]))

    def test_button_exit_keeps_camera_open_and_reentry_reacquires_frames(self):
        requests = [False, True, True, True, True, False,
                    False, True, True, True, True, False]
        code, records, ros, camera, _ = self.fake_run(
            [self.frame(i) for i in range(len(requests))], "--joystick",
            "--max-frames", str(len(requests)), requests=requests)
        self.assertEqual(code, 0)
        self.assertEqual(ros.enable.call_count, 2)
        self.assertEqual(ros.publish.call_count, 2)
        self.assertEqual(ros.disable.call_count, 2)
        self.assertEqual(camera.read.call_count, len(requests))
        for index in (3, 9):
            self.assertEqual(records[index]["tracking"]["reason"], "arming_wait_for_new_frame")
        for index in (0, 5, 6, 11):
            self.assertEqual(records[index]["tracking"]["mode"], "standby")

    def test_cancel_during_enable_never_publishes_motion(self):
        for reenter in (False, True):
            code, records, ros, camera, _ = self.fake_run(
                [self.frame(i) for i in range(4)], "--joystick", "--max-frames", "4",
                requests=[False, True, True, True], cancel_during_enable=True,
                reenter_during_enable=reenter)
            self.assertEqual(code, 0)
            ros.enable.assert_called_once()
            ros.disable.assert_called_once()
            ros.publish.assert_not_called()
            self.assertEqual(records[-2]["tracking"]["mode"], "standby")

    def test_disable_failure_keeps_camera_and_blocks_new_targets(self):
        requests = [False, True, True, True, True, False, True, True, True, True]
        code, records, ros, camera, _ = self.fake_run(
            [self.frame(i) for i in range(len(requests))], "--joystick",
            "--max-frames", str(len(requests)), requests=requests, disable_error="timeout")
        self.assertEqual(code, 0)
        self.assertEqual(camera.read.call_count, len(requests))
        ros.enable.assert_called_once()
        ros.publish.assert_called_once()
        for record in records[5:-1]:
            self.assertEqual(record["tracking"]["reason"], "disable_pending")
            self.assertEqual(record["tracking"]["mode"], "standby")

    def test_joystick_enable_failure_keeps_preview_and_requires_new_request(self):
        code, records, ros, camera, _ = self.fake_run(
            [self.frame(i) for i in range(5)], "--joystick", "--max-frames", "5",
            requests=[False, True, True, True, False], enable_error="controller unavailable")
        self.assertEqual(code, 0)
        ros.enable.assert_called_once()
        ros.require_joystick_release.assert_called_once()
        ros.publish.assert_not_called()
        self.assertEqual(camera.read.call_count, 5)
        self.assertEqual(records[3]["tracking"]["reason"], "enable_failed")
        self.assertEqual(records[3]["tracking"]["detail"], "controller unavailable")

    def test_capture_failure_stops_control_before_camera_cleanup(self):
        code, records, ros, camera, events = self.fake_run(
            [self.frame(0), TimeoutError("disconnected")], "--execute")
        self.assertEqual(code, 1)
        self.assertEqual(events, ["ros_close", "camera_stop"])
        self.assertEqual(records[-1]["tracking"]["reason"], "stopped")
        self.assertFalse(records[-1]["valid"])

    def test_keyboard_interrupt_releases_camera_and_ros(self):
        code, records, ros, camera, events = self.fake_run([KeyboardInterrupt()])
        self.assertEqual(code, 0)
        self.assertEqual(events, ["ros_close", "camera_stop"])

    def approach_frames(self, count):
        frames = [self.frame(i, distance=.2) for i in range(count)]
        for frame in frames:
            # The measured marker midpoint lies on the optical axis.
            frame["camera_matrix"][:2, 2] = [269.5, 139.5]
        return frames

    def test_approach_freezes_goal_and_keeps_reaching_after_markers_disappear(self):
        frames = self.approach_frames(30)
        for frame in frames[21:25]:
            frame["color_bgr"][:] = 255
            frame["depth_m"] = None
            frame["depth_status"] = "missing_depth"
        for frame in frames[25:]:
            frame["depth_m"][:] = .4  # A different observation must not retarget.
        code, records, ros, camera, _ = self.fake_run(
            frames, "--execute", "--tracking-mode", "approach", "--max-frames", "30",
            time_step=.0333)
        self.assertEqual(code, 0)
        self.assertFalse(camera.configuration["color_only"])
        self.assertTrue(ros.configuration["track_x"])
        self.assertTrue(ros.configuration["goal_mode"])
        self.assertEqual((camera.configuration["width"], camera.configuration["height"]), (640, 480))
        self.assertEqual((camera.configuration["depth_width"], camera.configuration["depth_height"]), (640, 360))
        ros.enable.assert_called_once()
        self.assertGreater(ros.publish.call_count, 0)
        for call in ros.publish.call_args_list:
            np.testing.assert_allclose(call.args[0], [.14, 0., .15], atol=1e-10)
        self.assertGreater(ros.publish_goal.call_count, 10)
        for call in ros.publish_goal.call_args_list:
            np.testing.assert_allclose(call.args[0], [.30, 0., .0755], atol=1e-10)
        goals = [r["tracking"] for r in records if r["tracking"].get("command_kind") == "goal"]
        self.assertEqual(len({g["goal_capture_timestamp_ms"] for g in goals}), 1)
        self.assertEqual(len({g["goal_capture_received_at"] for g in goals}), 1)
        for record in records[21:-1]:
            self.assertEqual(record["tracking"]["phase"], "approaching")
        self.assertFalse(records[21]["valid"])
        self.assertTrue(records[21]["tracking"]["valid"])

    def test_approach_feedback_loss_cancels_until_a_new_button_session(self):
        poses = [[.14, .15, 0.]] * 30
        poses[22] = None
        code, records, ros, _, _ = self.fake_run(
            self.approach_frames(30), "--execute", "--tracking-mode", "approach",
            "--max-frames", "30", time_step=.0333, poses=poses)
        self.assertEqual(code, 0)
        self.assertGreater(ros.publish_goal.call_count, 0)
        for record in records[22:-1]:
            self.assertEqual(record["tracking"]["phase"], "cancelled")
            self.assertFalse(record["tracking"]["valid"])
        commands = [call[0] for call in ros.method_calls if call[0] in ("publish_goal", "stop")]
        last_goal = len(commands) - 1 - commands[::-1].index("publish_goal")
        self.assertTrue(commands[last_goal + 1:])
        self.assertTrue(all(command == "stop" for command in commands[last_goal + 1:]))

    def test_approach_completion_holds_without_acquiring_another_target(self):
        poses = [[.14, .15, 0.]] * 22 + [[.30, .0755, 0.]] + [[.14, .15, 0.]] * 7
        code, records, ros, _, _ = self.fake_run(
            self.approach_frames(30), "--execute", "--tracking-mode", "approach",
            "--max-frames", "30", time_step=.0333, poses=poses)
        self.assertEqual(code, 0)
        self.assertGreater(ros.publish_goal.call_count, 0)
        for record in records[22:-1]:
            self.assertEqual(record["tracking"]["phase"], "complete")
            self.assertEqual(record["tracking"]["reason"], "goal_reached")
            self.assertFalse(record["tracking"]["valid"])

    def test_approach_button_exit_and_reentry_capture_a_new_goal(self):
        requests = [False] + [True] * 21 + [False] + [True] * 23 + [False]
        frames = self.approach_frames(len(requests))
        for frame in frames[23:]:
            frame["depth_m"][:] = .24
        code, records, ros, _, _ = self.fake_run(
            frames, "--joystick", "--tracking-mode", "approach",
            "--max-frames", str(len(frames)), requests=requests, time_step=.0333)
        self.assertEqual(code, 0)
        self.assertEqual(ros.enable.call_count, 2)
        self.assertEqual(ros.disable.call_count, 2)
        self.assertEqual(records[22]["tracking"]["mode"], "standby")
        first = [r["tracking"] for r in records[:22] if r["tracking"].get("command_kind") == "goal"]
        second = [r["tracking"] for r in records[23:] if r["tracking"].get("command_kind") == "goal"]
        self.assertTrue(first)
        self.assertTrue(second)
        np.testing.assert_allclose(first[-1]["goal_m"], [.30, 0., .0755], atol=1e-10)
        np.testing.assert_allclose(second[-1]["goal_m"], [.34, 0., .0755], atol=1e-10)
        self.assertGreater(second[0]["goal_capture_timestamp_ms"], first[-1]["goal_capture_timestamp_ms"])

    def test_approach_cancel_during_enable_never_sends_a_goal(self):
        code, records, ros, _, _ = self.fake_run(
            self.approach_frames(5), "--joystick", "--tracking-mode", "approach",
            "--max-frames", "5", requests=[False, True, True, True, True],
            time_step=.0333, cancel_during_enable=True)
        self.assertEqual(code, 0)
        ros.enable.assert_called_once()
        ros.disable.assert_called_once()
        ros.publish.assert_not_called()
        ros.publish_goal.assert_not_called()
        self.assertEqual(records[-2]["tracking"]["mode"], "standby")


if __name__ == "__main__":
    unittest.main()
