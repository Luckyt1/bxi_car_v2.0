#!/usr/bin/env python3
"""Recognize a QR or ArUco pair; never issue motion commands."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

# Source checkout and installed lib/chassis/vision share the same module API.
source_modules = Path(__file__).resolve().parents[1] / "thirdparty"
if (source_modules / "vision").is_dir():
    sys.path.insert(0, str(source_modules))


def parser():
    result = argparse.ArgumentParser(
        description="Gemini 335: QR/ArUco pair midpoint detection (no motor control).")
    result.add_argument("--image", type=Path, help="process one image without a camera or SDK")
    result.add_argument("--marker-type", choices=("qr", "aruco"), default="qr",
                        help="printed marker family (default: qr)")
    result.add_argument("--qr-texts", nargs=2, metavar=("FIRST", "SECOND"),
                        help="select two codes by their exact, distinct decoded contents")
    result.add_argument("--aruco-dictionary",
                        help="OpenCV ArUco dictionary name (default in ArUco mode: DICT_4X4_50)")
    result.add_argument("--marker-ids", nargs=2, type=int, metavar=("FIRST", "SECOND"),
                        help="select two distinct ArUco IDs")
    result.add_argument("--serial", help="Gemini 335 serial number; required with multiple cameras")
    result.add_argument("--color-only", action="store_true", help="disable depth; report pixels only")
    result.add_argument("--width", type=int, default=640)
    result.add_argument("--height", type=int, default=480)
    result.add_argument("--fps", type=int, default=30)
    result.add_argument("--timeout-ms", type=int, default=1000)
    result.add_argument("--headless", action="store_true", help="disable the preview window")
    result.add_argument("--max-frames", type=int, default=0, help="stop after N frames; 0 means unlimited")
    result.add_argument("--print-hz", type=float, default=5.0,
                        help="maximum console result frequency; every frame goes to --json-output")
    result.add_argument("--json-output", type=Path, help="write one JSON record per frame (overwrites)")
    result.add_argument("--output", type=Path, help="save annotated --image result")
    return result


def validate_options(args, argument_parser):
    for name in ("width", "height", "fps", "timeout_ms"):
        if getattr(args, name) <= 0:
            argument_parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.max_frames < 0:
        argument_parser.error("--max-frames must be nonnegative")
    if not math.isfinite(args.print_hz) or args.print_hz <= 0:
        argument_parser.error("--print-hz must be positive and finite")
    if args.output and not args.image:
        argument_parser.error("--output requires --image")
    paths = [path.resolve() for path in (args.image, args.output, args.json_output) if path]
    for index, path in enumerate(paths):
        for other in paths[index + 1:]:
            if path == other or (path.exists() and other.exists() and path.samefile(other)):
                argument_parser.error(
                    "input image, annotated output and JSON output must be different files")
    if args.image and not args.image.is_file():
        argument_parser.error(f"image does not exist: {args.image}")
    if args.qr_texts and (not all(args.qr_texts) or args.qr_texts[0] == args.qr_texts[1]):
        argument_parser.error("--qr-texts requires two different nonempty strings")
    if args.marker_type == "qr" and (args.marker_ids is not None or args.aruco_dictionary is not None):
        argument_parser.error("--marker-ids and --aruco-dictionary require --marker-type aruco")
    if args.marker_type == "aruco" and args.qr_texts is not None:
        argument_parser.error("--qr-texts requires --marker-type qr")
    if args.marker_ids is not None and (
            min(args.marker_ids) < 0 or args.marker_ids[0] == args.marker_ids[1]):
        argument_parser.error("--marker-ids requires two different nonnegative IDs")


def unavailable_position(status):
    return dict(valid=False, status=status, frame_id="gemini_color_optical_frame",
                centers_m=None, midpoint_m=None)


def failure_record(source, status, error):
    return dict(schema_version=1, source=source, timestamp_unix_s=time.time(),
                device_timestamp_ms=None, image_size=None, valid=False, status=status,
                codes=[], pair_indices=[], midpoint_px=None,
                position_3d=unavailable_position(status), error=str(error))


def process_image(detector, image, source, frame=None):
    result = detector.detect(image)
    result.update(schema_version=1, source=source, timestamp_unix_s=time.time(),
                  device_timestamp_ms=frame["timestamp_ms"] if frame else None,
                  image_size=[int(image.shape[1]), int(image.shape[0])])
    if frame is None:
        result["position_3d"] = unavailable_position("image_has_no_depth")
    else:
        from vision.gemini_camera import locate_midpoint
        result["position_3d"] = locate_midpoint(result, frame)
    return result


def draw(image, result):
    import cv2
    from vision.qr_midpoint import annotate
    rendered = annotate(image, result)
    position = result["position_3d"]
    if position["valid"]:
        x, y, z = position["midpoint_m"]
        text = f"Camera midpoint (m): {x:.3f}, {y:.3f}, {z:.3f}"
    else:
        text = f"3D: {position['status']}"
    cv2.putText(rendered, text, (10, max(20, image.shape[0] - 15)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 180, 255), 1, cv2.LINE_AA)
    return rendered


def run(args):
    import cv2
    from vision.qr_midpoint import ArucoMidpointDetector, QrMidpointDetector

    detector = (ArucoMidpointDetector(
        dictionary=args.aruco_dictionary or "DICT_4X4_50", marker_ids=args.marker_ids or ())
        if args.marker_type == "aruco" else QrMidpointDetector(args.qr_texts or ()))
    window_name = "ArUco pair midpoint" if args.marker_type == "aruco" else "QR pair midpoint"
    display = not args.headless and bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    if not args.headless and not display:
        print("No desktop display found; using headless mode.", file=sys.stderr)
    output = None
    camera = None
    last_print = -math.inf
    last_valid = None
    source = "image" if args.image else "gemini335"

    def emit(record, force=False):
        nonlocal last_print, last_valid
        line = json.dumps(record, ensure_ascii=False, allow_nan=False)
        if output is not None:
            output.write(line + "\n")
            output.flush()
        now = time.monotonic()
        state = (record["valid"], record["status"], record["position_3d"]["valid"],
                 record["position_3d"]["status"])
        if force or state != last_valid or now - last_print >= 1.0 / args.print_hz:
            print(line, flush=True)
            last_print = now
        last_valid = state

    try:
        if args.json_output:
            output = args.json_output.open("w", encoding="utf-8")
        if args.image:
            image = cv2.imread(str(args.image))
            if image is None:
                raise ValueError(f"cannot read image: {args.image}")
            result = process_image(detector, image, source)
            emit(result, force=True)
            if args.output or display:
                rendered = draw(image, result)
                if args.output and not cv2.imwrite(str(args.output), rendered):
                    raise OSError(f"cannot write annotated image: {args.output}")
                if display:
                    cv2.imshow(window_name, rendered)
                    cv2.waitKey(0)
            return 0

        from vision.gemini_camera import GeminiCamera
        camera = GeminiCamera(serial=args.serial, color_only=args.color_only,
                              width=args.width, height=args.height, fps=args.fps,
                              timeout_ms=args.timeout_ms)
        metadata = camera.start()
        print("Gemini connected: " + json.dumps(metadata, ensure_ascii=False), file=sys.stderr)
        count = 0
        while not args.max_frames or count < args.max_frames:
            frame = camera.read()
            result = process_image(detector, frame["color_bgr"], source, frame)
            emit(result)
            count += 1
            if display:
                cv2.imshow(window_name, draw(frame["color_bgr"], result))
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    break
        emit(failure_record(source, "stopped", "recognition session ended"), force=True)
        return 0
    except KeyboardInterrupt:
        emit(failure_record(source, "stopped", "interrupted"), force=True)
        return 0
    except Exception as error:
        emit(failure_record(source, "camera_timeout" if isinstance(error, TimeoutError)
                            else "error", error), force=True)
        print(f"QR recognition failed: {error}", file=sys.stderr)
        return 1
    finally:
        try:
            if camera is not None:
                camera.stop()
        finally:
            if output is not None:
                output.close()
            if display:
                cv2.destroyAllWindows()


def main(argv=None):
    argument_parser = parser()
    args = argument_parser.parse_args(argv)
    validate_options(args, argument_parser)
    try:
        return run(args)
    except Exception as error:
        print(f"QR recognition unavailable: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
