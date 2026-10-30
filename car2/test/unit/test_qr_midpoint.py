"""Offline QR recognition checks using real generated images and geometry cases."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import unittest
from unittest import mock

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "qr_midpoint", ROOT / "src" / "thirdparty" / "vision" / "qr_midpoint.py"
)
vision = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(vision)


def qr_scene(texts):
    image = np.full((320, max(1, len(texts)) * 300, 3), 255, dtype=np.uint8)
    for index, text in enumerate(texts):
        qr = cv2.QRCodeEncoder_create().encode(text)
        qr = cv2.copyMakeBorder(qr, 4, 4, 4, 4, cv2.BORDER_CONSTANT, value=255)
        qr = cv2.resize(qr, (240, 240), interpolation=cv2.INTER_NEAREST)
        image[40:280, 30 + index * 300:270 + index * 300] = cv2.cvtColor(
            qr, cv2.COLOR_GRAY2BGR
        )
    return image


def aruco_scene(ids, dictionary="DICT_4X4_50"):
    image = np.full((320, max(1, len(ids)) * 300, 3), 255, dtype=np.uint8)
    aruco_dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    for index, marker_id in enumerate(ids):
        marker = cv2.aruco.generateImageMarker(aruco_dictionary, marker_id, 240)
        image[40:280, 30 + index * 300:270 + index * 300] = cv2.cvtColor(
            marker, cv2.COLOR_GRAY2BGR
        )
    return image


class QrMidpointTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not hasattr(cv2, "QRCodeEncoder_create"):
            raise RuntimeError("OpenCV QRCodeEncoder_create is required for real QR tests")
        cls.blank = qr_scene([])
        cls.single = qr_scene(["left"])
        cls.pair = qr_scene(["left", "right"])
        cls.triple = qr_scene(["left", "right", "extra"])

    def assert_invalid(self, result, status):
        self.assertFalse(result["valid"])
        self.assertEqual(result["status"], status)
        self.assertEqual(result["pair_indices"], [])
        self.assertIsNone(result["midpoint_px"])
        json.dumps(result, allow_nan=False)

    def detector_with_points(self, points, texts=(), success=True):
        detector = vision.QrMidpointDetector()
        detector._detector = mock.Mock()
        detector._detector.detectAndDecodeMulti.return_value = (
            success, texts, np.asarray(points, dtype=np.float64), ()
        )
        return detector

    def test_blank_image(self):
        self.assert_invalid(vision.QrMidpointDetector().detect(self.blank), "no_codes")

    def test_single_code_does_not_produce_target(self):
        result = vision.QrMidpointDetector().detect(self.single)
        self.assert_invalid(result, "not_enough_codes")
        self.assertEqual([code["text"] for code in result["codes"]], ["left"])

    def test_real_pair_and_json_geometry(self):
        result = vision.QrMidpointDetector().detect(self.pair)
        self.assertTrue(result["valid"])
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["pair_indices"], [0, 1])
        self.assertEqual([code["text"] for code in result["codes"]], ["left", "right"])
        np.testing.assert_allclose(result["midpoint_px"], [300, 160], atol=2)
        json.dumps(result, allow_nan=False)

    def test_more_than_two_codes_is_ambiguous(self):
        result = vision.QrMidpointDetector().detect(self.triple)
        self.assert_invalid(result, "ambiguous_pair")
        self.assertEqual(len(result["codes"]), 3)

    def test_requested_text_pair_ignores_unrelated_codes_and_preserves_selection_order(self):
        result = vision.QrMidpointDetector(("extra", "left")).detect(self.triple)
        self.assertTrue(result["valid"])
        self.assertEqual(result["pair_indices"], [2, 0])
        np.testing.assert_allclose(result["midpoint_px"], [450, 160], atol=2)

    def test_missing_requested_text_invalidates_pair(self):
        result = vision.QrMidpointDetector(("left", "missing")).detect(self.pair)
        self.assert_invalid(result, "target_missing")

    def test_duplicate_requested_text_is_ambiguous(self):
        result = vision.QrMidpointDetector(("left", "right")).detect(
            qr_scene(["left", "left", "right"])
        )
        self.assert_invalid(result, "target_ambiguous")

    def test_default_selection_accepts_two_identical_payloads(self):
        result = vision.QrMidpointDetector().detect(qr_scene(["same", "same"]))
        self.assertTrue(result["valid"])
        self.assertEqual([code["text"] for code in result["codes"]], ["same", "same"])

    def test_invalid_frame_does_not_reuse_previous_target(self):
        detector = vision.QrMidpointDetector()
        self.assertTrue(detector.detect(self.pair)["valid"])
        self.assert_invalid(detector.detect(self.blank), "no_codes")

    def test_invalid_selection_configuration(self):
        for texts in [None, "ab", ("a",), ("a", "a"), ("a", ""), ("a", 2),
                      ("a", "b", "c")]:
            with self.subTest(texts=texts), self.assertRaises(ValueError):
                vision.QrMidpointDetector(texts)

    def test_invalid_image_contract(self):
        for image in [None, np.zeros((0, 10, 3), np.uint8),
                      np.zeros((20, 20), np.uint8), np.zeros((20, 20, 4), np.uint8),
                      np.zeros((20, 20, 3), np.float32)]:
            with self.subTest(shape=getattr(image, "shape", None)), self.assertRaises(ValueError):
                vision.QrMidpointDetector().detect(image)

    def test_perspective_center_is_diagonal_intersection(self):
        square = np.float32([[0, 0], [100, 0], [100, 100], [0, 100]])
        quadrilateral = np.float32([[20, 30], [180, 40], [130, 180], [40, 150]])
        transform = cv2.getPerspectiveTransform(square, quadrilateral)
        expected = cv2.perspectiveTransform(np.float32([[[50, 50]]]), transform)[0, 0]
        other = np.float32([[220, 40], [280, 40], [280, 100], [220, 100]])
        result = self.detector_with_points([quadrilateral, other]).detect(self.blank)
        self.assertTrue(result["valid"])
        np.testing.assert_allclose(result["codes"][0]["center_px"], expected, atol=1e-5)
        self.assertGreater(np.linalg.norm(expected - quadrilateral.mean(axis=0)), 5)
        np.testing.assert_allclose(result["midpoint_px"], (expected + [250, 70]) / 2,
                                   atol=1e-5)

    def test_clockwise_and_counterclockwise_corners_are_accepted(self):
        corners = [[10, 10], [80, 10], [80, 80], [10, 80]]
        result = self.detector_with_points([corners, list(reversed(corners))]).detect(self.blank)
        self.assertTrue(result["valid"])
        self.assertEqual(result["midpoint_px"], [45, 45])

    def test_bad_boxes_are_rejected_before_pairing(self):
        good = [[200, 10], [280, 10], [280, 80], [200, 80]]
        bad_boxes = [
            [[10, 10], [80, 10], [80, np.nan], [10, 80]],
            [[10, 10], [80, 10], [80, np.inf], [10, 80]],
            [[10, 10], [10, 10], [10, 10], [10, 10]],
            [[10, 10], [20, 20], [30, 30], [40, 40]],
            [[10, 10], [80, 10], [20, 20], [10, 80]],  # Concave.
            [[10, 10], [80, 80], [80, 10], [10, 80]],  # Self-intersection.
            [[-1, 10], [80, 10], [80, 80], [10, 80]],
            [[10, 10], [300, 10], [300, 80], [10, 80]],
            [[10, 10], [80, 10], [80, 320], [10, 80]],
        ]
        for corners in bad_boxes:
            with self.subTest(corners=corners):
                result = self.detector_with_points([corners, good]).detect(self.blank)
                self.assert_invalid(result, "not_enough_codes")
                self.assertEqual(len(result["codes"]), 1)

    def test_decode_false_keeps_detected_unlabeled_boxes(self):
        corners = [[[10, 10], [80, 10], [80, 80], [10, 80]],
                   [[200, 10], [280, 10], [280, 80], [200, 80]]]
        detector = self.detector_with_points(corners, success=False)
        result = detector.detect(self.blank)
        self.assertTrue(result["valid"])
        self.assertEqual([code["text"] for code in result["codes"]], ["", ""])
        detector._detector.detectMulti.assert_not_called()
        detector.qr_texts = ("left", "right")
        self.assert_invalid(detector.detect(self.blank), "target_missing")

    def test_decode_exception_and_missing_points_fall_back_to_detection(self):
        corners = np.float32([[[10, 10], [80, 10], [80, 80], [10, 80]],
                              [[200, 10], [280, 10], [280, 80], [200, 80]]])
        for exception in [None, cv2.error("decode failed"),
                          UnicodeDecodeError("utf8", b"\xff", 0, 1, "invalid")]:
            with self.subTest(exception=exception):
                detector = vision.QrMidpointDetector()
                detector._detector = mock.Mock()
                detector._detector.detectAndDecodeMulti.return_value = (False, (), None, ())
                detector._detector.detectAndDecodeMulti.side_effect = exception
                detector._detector.detectMulti.return_value = (True, corners)
                self.assertTrue(detector.detect(self.blank)["valid"])
                detector._detector.detectMulti.assert_called_once()

    def test_annotation_preserves_input_and_marks_midpoint(self):
        before = self.pair.copy()
        result = vision.QrMidpointDetector().detect(self.pair)
        annotated = vision.annotate(self.pair, result)
        np.testing.assert_array_equal(self.pair, before)
        self.assertFalse(np.shares_memory(self.pair, annotated))
        self.assertEqual(annotated.shape, self.pair.shape)
        np.testing.assert_array_equal(annotated[160, 300], [0, 0, 255])


class ArucoMidpointTests(unittest.TestCase):
    def assert_invalid(self, result, status):
        self.assertFalse(result["valid"])
        self.assertEqual(result["status"], status)
        self.assertEqual(result["pair_indices"], [])
        self.assertIsNone(result["midpoint_px"])
        json.dumps(result, allow_nan=False)

    def test_real_pair_and_json_geometry(self):
        result = vision.ArucoMidpointDetector().detect(aruco_scene([0, 1]))
        self.assertTrue(result["valid"])
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["pair_indices"], [0, 1])
        self.assertEqual([code["id"] for code in result["codes"]], [0, 1])
        self.assertEqual([code["text"] for code in result["codes"]], ["0", "1"])
        self.assertTrue(all(code["type"] == "aruco" for code in result["codes"]))
        self.assertTrue(all(code["dictionary"] == "DICT_4X4_50"
                            for code in result["codes"]))
        np.testing.assert_allclose(result["midpoint_px"], [299.5, 159.5], atol=1)
        json.dumps(result, allow_nan=False)

    def test_blank_and_single_code_do_not_produce_target(self):
        detector = vision.ArucoMidpointDetector()
        self.assert_invalid(detector.detect(aruco_scene([])), "no_codes")
        self.assert_invalid(detector.detect(aruco_scene([0])), "not_enough_codes")

    def test_more_than_two_codes_is_ambiguous(self):
        result = vision.ArucoMidpointDetector().detect(aruco_scene([0, 1, 2]))
        self.assert_invalid(result, "ambiguous_pair")
        self.assertEqual(len(result["codes"]), 3)

    def test_requested_ids_ignore_unrelated_codes_and_preserve_selection_order(self):
        result = vision.ArucoMidpointDetector(marker_ids=(2, 0)).detect(
            aruco_scene([0, 1, 2])
        )
        self.assertTrue(result["valid"])
        self.assertEqual(result["pair_indices"], [2, 0])
        np.testing.assert_allclose(result["midpoint_px"], [449.5, 159.5], atol=1)

    def test_duplicate_ids_are_ambiguous(self):
        self.assert_invalid(
            vision.ArucoMidpointDetector().detect(aruco_scene([0, 0])),
            "target_ambiguous",
        )
        self.assert_invalid(
            vision.ArucoMidpointDetector(marker_ids=(0, 1)).detect(aruco_scene([0, 0, 1])),
            "target_ambiguous",
        )

    def test_missing_requested_id_invalidates_pair(self):
        result = vision.ArucoMidpointDetector(marker_ids=(0, 2)).detect(aruco_scene([0, 1]))
        self.assert_invalid(result, "target_missing")

    def test_invalid_selection_configuration(self):
        for ids in [None, "", b"", "01", (0,), (0, 0), (-1, 1), (0, 50), (0, 1.0),
                    (False, 1), (0, 1, 2)]:
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                vision.ArucoMidpointDetector(marker_ids=ids)

    def test_predefined_dictionary_and_its_id_limit(self):
        result = vision.ArucoMidpointDetector("DICT_5X5_100", (0, 99)).detect(
            aruco_scene([0, 99], "DICT_5X5_100")
        )
        self.assertTrue(result["valid"])
        self.assertEqual([code["dictionary"] for code in result["codes"]],
                         ["DICT_5X5_100", "DICT_5X5_100"])
        with self.assertRaises(ValueError):
            vision.ArucoMidpointDetector("DICT_5X5_100", (0, 100))

    def test_invalid_dictionary(self):
        for dictionary in [None, 0, "", "DICT_DOES_NOT_EXIST", "getPredefinedDictionary"]:
            with self.subTest(dictionary=dictionary), self.assertRaises(ValueError):
                vision.ArucoMidpointDetector(dictionary)

    def test_invalid_image_contract(self):
        for image in [None, np.zeros((0, 10, 3), np.uint8),
                      np.zeros((20, 20), np.uint8), np.zeros((20, 20, 4), np.uint8),
                      np.zeros((20, 20, 3), np.float32)]:
            with self.subTest(shape=getattr(image, "shape", None)), self.assertRaises(ValueError):
                vision.ArucoMidpointDetector().detect(image)

    def test_invalid_frame_does_not_reuse_previous_target(self):
        detector = vision.ArucoMidpointDetector()
        self.assertTrue(detector.detect(aruco_scene([0, 1]))["valid"])
        self.assert_invalid(detector.detect(aruco_scene([0])), "not_enough_codes")

    def test_annotation_preserves_input_and_displays_ids(self):
        scene = aruco_scene([0, 1])
        before = scene.copy()
        result = vision.ArucoMidpointDetector().detect(scene)
        with mock.patch.object(cv2, "putText", wraps=cv2.putText) as put_text:
            annotated = vision.annotate(scene, result)
        labels = [call.args[1] for call in put_text.call_args_list]
        self.assertIn("ID 0", labels)
        self.assertIn("ID 1", labels)
        np.testing.assert_array_equal(scene, before)
        self.assertFalse(np.shares_memory(scene, annotated))
        np.testing.assert_array_equal(annotated[160, 300], [0, 0, 255])


if __name__ == "__main__":
    unittest.main()
