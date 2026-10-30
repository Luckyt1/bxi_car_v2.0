"""Detect the image-space midpoint between two QR or ArUco marker centers.

Coordinates are pixels in the input color image, not robot coordinates. A marker
center is the intersection of its corner diagonals, including under perspective.
The output midpoint is the arithmetic mean of those two image-space centers.
"""

from __future__ import annotations

import cv2
import numpy as np


def _cross(a, b):
    return float(a[0] * b[1] - a[1] * b[0])


def _validated_corners(corners, width, height):
    """Return finite convex corners and their diagonal intersection, or None."""
    try:
        points = np.asarray(corners, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if points.shape != (4, 2) or not np.isfinite(points).all():
        return None
    if (points < 0).any() or (points[:, 0] > width - 1).any() or (
        points[:, 1] > height - 1
    ).any():
        return None

    edges = np.roll(points, -1, axis=0) - points
    turns = np.array([_cross(edges[i], edges[(i + 1) % 4]) for i in range(4)])
    if not ((turns > 1e-6).all() or (turns < -1e-6).all()):
        return None

    first = points[2] - points[0]
    second = points[3] - points[1]
    denominator = _cross(first, second)
    if abs(denominator) <= 1e-6:
        return None
    offset = points[1] - points[0]
    first_fraction = _cross(offset, second) / denominator
    second_fraction = _cross(offset, first) / denominator
    if not (0 < first_fraction < 1 and 0 < second_fraction < 1):
        return None
    center = points[0] + first_fraction * first
    return points.tolist(), center.tolist()


def _validate_image(image_bgr):
    if (
        not isinstance(image_bgr, np.ndarray)
        or image_bgr.dtype != np.uint8
        or image_bgr.ndim != 3
        or image_bgr.shape[2] != 3
        or image_bgr.shape[0] == 0
        or image_bgr.shape[1] == 0
    ):
        raise ValueError("image_bgr must be a nonempty uint8 HxWx3 BGR image")


def _pair_result(codes, targets=(), selection_key="text"):
    # OpenCV's detection order is not stable; provide left-to-right indexing.
    codes.sort(key=lambda code: tuple(code["center_px"]))
    result = {
        "valid": False,
        "status": "no_codes",
        "codes": codes,
        "pair_indices": [],
        "midpoint_px": None,
    }
    if targets:
        matches = [
            [index for index, code in enumerate(codes) if code[selection_key] == target]
            for target in targets
        ]
        if any(len(indices) > 1 for indices in matches):
            result["status"] = "target_ambiguous"
            return result
        if any(not indices for indices in matches):
            result["status"] = "target_missing"
            return result
        pair = [indices[0] for indices in matches]
    else:
        if len(codes) < 2:
            result["status"] = "no_codes" if not codes else "not_enough_codes"
            return result
        if len(codes) > 2:
            result["status"] = "ambiguous_pair"
            return result
        if selection_key == "id" and codes[0]["id"] == codes[1]["id"]:
            result["status"] = "target_ambiguous"
            return result
        pair = [0, 1]

    first, second = (codes[index]["center_px"] for index in pair)
    result.update({
        "valid": True,
        "status": "ok",
        "pair_indices": pair,
        "midpoint_px": [(first[0] + second[0]) / 2, (first[1] + second[1]) / 2],
    })
    return result


class QrMidpointDetector:
    """Find one unambiguous pair, optionally selected by exact decoded texts."""

    def __init__(self, qr_texts=()):
        if isinstance(qr_texts, (str, bytes)):
            raise ValueError("qr_texts must contain two distinct nonempty strings")
        try:
            self.qr_texts = tuple(qr_texts)
        except TypeError as exc:
            raise ValueError("qr_texts must be an iterable of strings") from exc
        if self.qr_texts and (
            len(self.qr_texts) != 2
            or any(not isinstance(text, str) or not text for text in self.qr_texts)
            or self.qr_texts[0] == self.qr_texts[1]
        ):
            raise ValueError("qr_texts must contain two distinct nonempty strings")
        self._detector = cv2.QRCodeDetector()

    def detect(self, image_bgr):
        """Return a fresh JSON-serializable result; never reuse an earlier target."""
        _validate_image(image_bgr)
        codes = []
        texts, points = (), None
        try:
            _, texts, points, _ = self._detector.detectAndDecodeMulti(image_bgr)
        except (cv2.error, UnicodeDecodeError):
            # Some OpenCV builds cannot decode certain QR payloads. Geometric
            # detection remains useful when a text-selected pair is not required.
            pass
        if points is None or len(points) == 0:
            _, points = self._detector.detectMulti(image_bgr)
            texts = ()
        if points is None:
            points = ()
        if texts is None:
            texts = ()

        height, width = image_bgr.shape[:2]
        for index, corners in enumerate(points):
            geometry = _validated_corners(corners, width, height)
            if geometry is None:
                continue
            corners_px, center_px = geometry
            text = texts[index] if index < len(texts) else ""
            codes.append({
                "text": text if isinstance(text, str) else "",
                "corners_px": corners_px,
                "center_px": center_px,
            })

        return _pair_result(codes, self.qr_texts)


class ArucoMidpointDetector:
    """Find a pair of ArUco markers, optionally selected by dictionary IDs."""

    def __init__(self, dictionary="DICT_4X4_50", marker_ids=()):
        dictionary_id = (
            getattr(cv2.aruco, dictionary, None)
            if isinstance(dictionary, str) and dictionary.startswith("DICT_")
            else None
        )
        if not isinstance(dictionary_id, int):
            raise ValueError("dictionary must name a predefined ArUco DICT_* dictionary")
        try:
            aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        except cv2.error as exc:
            raise ValueError(f"invalid ArUco dictionary: {dictionary}") from exc

        if isinstance(marker_ids, (str, bytes)):
            raise ValueError("marker_ids must contain two distinct nonnegative integers")
        try:
            marker_ids = tuple(marker_ids)
        except TypeError as exc:
            raise ValueError("marker_ids must contain two distinct nonnegative integers") from exc
        count = aruco_dictionary.bytesList.shape[0]
        if marker_ids and (
            len(marker_ids) != 2
            or any(isinstance(value, bool) or not isinstance(value, (int, np.integer))
                   or value < 0 or value >= count for value in marker_ids)
            or marker_ids[0] == marker_ids[1]
        ):
            raise ValueError(
                f"marker_ids must contain two distinct integers between 0 and {count - 1}"
            )
        self.dictionary = dictionary
        self.marker_ids = tuple(int(value) for value in marker_ids)
        parameters = cv2.aruco.DetectorParameters()
        parameters.perspectiveRemovePixelPerCell = 8
        self._detector = cv2.aruco.ArucoDetector(aruco_dictionary, parameters)

    def detect(self, image_bgr):
        """Return current-frame marker geometry with no inferred or stale target."""
        _validate_image(image_bgr)
        corners, ids, _ = self._detector.detectMarkers(image_bgr)
        codes = []
        if ids is not None:
            height, width = image_bgr.shape[:2]
            for points, marker_id in zip(corners, ids.ravel()):
                geometry = _validated_corners(
                    np.asarray(points).reshape(4, 2), width, height
                )
                if geometry is None:
                    continue
                corners_px, center_px = geometry
                codes.append({
                    "id": int(marker_id),
                    "type": "aruco",
                    "dictionary": self.dictionary,
                    "text": str(int(marker_id)),
                    "corners_px": corners_px,
                    "center_px": center_px,
                })
        return _pair_result(codes, self.marker_ids, selection_key="id")


def annotate(image_bgr, result):
    """Draw detection geometry on a new BGR image without modifying the input."""
    image = image_bgr.copy()
    pair = result["pair_indices"]
    for index, code in enumerate(result["codes"]):
        corners = np.rint(code["corners_px"]).astype(np.int32)
        center = tuple(np.rint(code["center_px"]).astype(int))
        color = (0, 200, 0) if index in pair else (0, 165, 255)
        cv2.polylines(image, [corners], True, color, 2, cv2.LINE_AA)
        cv2.circle(image, center, 4, color, -1, cv2.LINE_AA)
        label = f"ID {code['id']}" if code.get("type") == "aruco" else f"QR {index + 1}"
        cv2.putText(image, label, (center[0] + 6, center[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    if result["valid"]:
        centers = [
            tuple(np.rint(result["codes"][index]["center_px"]).astype(int))
            for index in pair
        ]
        midpoint = tuple(np.rint(result["midpoint_px"]).astype(int))
        cv2.line(image, centers[0], centers[1], (0, 200, 0), 2, cv2.LINE_AA)
        cv2.drawMarker(image, midpoint, (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(image, f"midpoint ({midpoint[0]}, {midpoint[1]})",
                    (midpoint[0] + 8, midpoint[1] + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
    cv2.putText(image, result["status"], (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 200, 0) if result["valid"] else (0, 165, 255), 2, cv2.LINE_AA)
    return image
