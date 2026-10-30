"""Exercise the camera-independent entrypoint and camera lifecycle with fake frames."""

import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
ENTRY = ROOT / "src/example/qr_midpoint.py"
spec = importlib.util.spec_from_file_location("qr_midpoint_entry_test", ENTRY)
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)


def pair_image():
    image = np.full((300, 600, 3), 255, dtype=np.uint8)
    encoder = cv2.QRCodeEncoder_create()
    for x, text in ((40, "left-marker"), (340, "right-marker")):
        code = encoder.encode(text)
        code = cv2.resize(code, (200, 200), interpolation=cv2.INTER_NEAREST)
        image[50:250, x:x + 200] = cv2.cvtColor(code, cv2.COLOR_GRAY2BGR)
    return image


class EntrypointTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="qr midpoint cli ")
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.input = self.directory / "two codes.png"
        self.assertTrue(cv2.imwrite(str(self.input), pair_image()))

    def launch(self, *arguments, entrypoint=ENTRY):
        return subprocess.run([sys.executable, "-B", str(entrypoint), *map(str, arguments)],
                              cwd=self.directory, text=True, capture_output=True, timeout=15)

    def test_offline_detects_real_pair_and_saves_annotation_and_json(self):
        output = self.directory / "annotated.png"
        records = self.directory / "result.jsonl"
        result = self.launch("--image", self.input, "--headless", "--output", output,
                             "--json-output", records)
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(result.stdout)
        self.assertTrue(data["valid"])
        self.assertEqual(data["source"], "image")
        self.assertEqual(data["image_size"], [600, 300])
        self.assertEqual({code["text"] for code in data["codes"]},
                         {"left-marker", "right-marker"})
        self.assertAlmostEqual(data["midpoint_px"][0], 289.5, delta=2)
        self.assertAlmostEqual(data["midpoint_px"][1], 149.5, delta=2)
        self.assertFalse(data["position_3d"]["valid"])
        self.assertIsNone(data["position_3d"]["midpoint_m"])
        self.assertEqual(json.loads(records.read_text()), data)
        self.assertEqual(cv2.imread(str(output)).shape, (300, 600, 3))
        self.assertFalse(np.array_equal(cv2.imread(str(output)), cv2.imread(str(self.input))))

    def test_missing_selected_code_is_invalid_not_old_target(self):
        result = self.launch("--image", self.input, "--headless",
                             "--qr-texts", "left-marker", "not-visible")
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(result.stdout)
        self.assertFalse(data["valid"])
        self.assertIsNone(data["midpoint_px"])

    def test_aruco_mode_detects_selected_ids_and_saves_annotation(self):
        image = np.full((300, 600, 3), 255, dtype=np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        for x, marker_id in ((40, 0), (340, 1)):
            marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 200)
            image[50:250, x:x + 200] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        self.assertTrue(cv2.imwrite(str(self.input), image))
        output = self.directory / "aruco-annotated.png"
        result = self.launch("--image", self.input, "--headless", "--marker-type", "aruco",
                             "--aruco-dictionary", "DICT_4X4_50", "--marker-ids", "0", "1",
                             "--output", output)
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(result.stdout)
        self.assertTrue(data["valid"])
        self.assertEqual([code["id"] for code in data["codes"]], [0, 1])
        self.assertAlmostEqual(data["midpoint_px"][0], 289.5, delta=2)
        self.assertAlmostEqual(data["midpoint_px"][1], 149.5, delta=2)
        self.assertEqual(cv2.imread(str(output)).shape, image.shape)
        self.assertFalse(data["position_3d"]["valid"])

    def test_help_needs_neither_sdk_nor_opencv(self):
        result = subprocess.run([sys.executable, "-S", "-B", str(ENTRY), "--help"],
                                text=True, capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("no motor control", result.stdout)

    def test_bad_arguments_are_rejected_before_camera_access(self):
        for arguments in [("--fps", "0"), ("--width", "-1"), ("--max-frames", "-1"),
                          ("--print-hz", "nan"), ("--timeout-ms", "0"),
                          ("--qr-texts", "same", "same"),
                          ("--marker-ids", "0", "1"),
                          ("--aruco-dictionary", "DICT_4X4_50"),
                          ("--marker-type", "aruco", "--qr-texts", "a", "b"),
                          ("--marker-type", "aruco", "--marker-ids", "1", "1"),
                          ("--marker-type", "aruco", "--marker-ids", "-1", "1"),
                          ("--output", str(self.directory / "unused.png"))]:
            with self.subTest(arguments=arguments):
                self.assertEqual(self.launch(*arguments).returncode, 2)

    def test_input_cannot_be_overwritten_by_output(self):
        before = self.input.read_bytes()
        for option in ("--output", "--json-output"):
            result = self.launch("--image", self.input, option, self.input, "--headless")
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertEqual(self.input.read_bytes(), before)

    def test_invalid_image_emits_explicit_invalid_result(self):
        self.input.write_bytes(b"not an image")
        result = self.launch("--image", self.input, "--headless")
        self.assertEqual(result.returncode, 1)
        self.assertFalse(json.loads(result.stdout)["valid"])

    def test_hard_link_collisions_do_not_modify_input_or_outputs(self):
        before = self.input.read_bytes()
        alias = self.directory / "alias.png"
        os.link(self.input, alias)
        for option in ("--output", "--json-output"):
            result = self.launch("--image", self.input, option, alias, "--headless")
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertEqual(self.input.read_bytes(), before)
        first = self.directory / "first.png"
        second = self.directory / "second.jsonl"
        first.write_bytes(b"existing output")
        os.link(first, second)
        result = self.launch("--image", self.input, "--headless",
                             "--output", first, "--json-output", second)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertEqual(first.read_bytes(), b"existing output")

    def test_installed_layout_runs_from_unrelated_directory(self):
        installed = self.directory / "install/lib/chassis"
        installed.mkdir(parents=True)
        shutil.copy2(ENTRY, installed / ENTRY.name)
        shutil.copytree(ROOT / "src/thirdparty/vision", installed / "vision",
                        ignore=shutil.ignore_patterns("__pycache__"))
        result = self.launch("--image", self.input, "--headless",
                             entrypoint=installed / ENTRY.name)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(json.loads(result.stdout)["valid"])

    def test_shell_launcher_preserves_paths_with_spaces(self):
        result = subprocess.run(
            ["bash", str(ROOT / "start_qr_midpoint.sh"), "--image", str(self.input), "--headless"],
            env=dict(os.environ, CHASSIS_VISION_PYTHON=sys.executable),
            cwd=self.directory, text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(json.loads(result.stdout)["valid"])

    def fake_camera_run(self, frames, *arguments):
        camera = mock.Mock()
        camera.start.return_value = dict(name="Gemini 335", serial="fake")
        camera.read.side_effect = frames
        output = io.StringIO()
        with mock.patch("vision.gemini_camera.GeminiCamera", return_value=camera), \
                contextlib.redirect_stdout(output), contextlib.redirect_stderr(io.StringIO()):
            status = entry.main(["--headless", *arguments])
        camera.stop.assert_called_once()
        return status, [json.loads(line) for line in output.getvalue().splitlines()]

    @staticmethod
    def frame():
        return dict(color_bgr=pair_image(), depth_m=None, camera_matrix=None, distortion=None,
                    timestamp_ms=123.0, depth_status="depth_disabled")

    def test_camera_timeout_invalidates_result_and_releases_camera(self):
        status, records = self.fake_camera_run([self.frame(), TimeoutError("unplugged")])
        self.assertEqual(status, 1)
        self.assertTrue(records[0]["valid"])
        self.assertEqual(records[0]["device_timestamp_ms"], 123.0)
        self.assertFalse(records[-1]["valid"])
        self.assertEqual(records[-1]["status"], "camera_timeout")
        self.assertIsNone(records[-1]["midpoint_px"])
        self.assertIsNone(records[-1]["position_3d"]["midpoint_m"])

    def test_frame_limit_and_interrupt_report_session_end(self):
        for frames, arguments in [([self.frame()], ("--max-frames", "1")),
                                  ([KeyboardInterrupt()], ())]:
            with self.subTest(arguments=arguments):
                status, records = self.fake_camera_run(frames, *arguments)
                self.assertEqual(status, 0)
                self.assertEqual(records[-1]["status"], "stopped")
                self.assertFalse(records[-1]["valid"])


if __name__ == "__main__":
    unittest.main()
