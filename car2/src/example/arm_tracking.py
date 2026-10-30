#!/usr/bin/env python3
"""Preview visual targets, hold depth or approach a recorded goal with explicit control."""

import json
import math
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
import qr_midpoint as recognition


def default_config():
    source = Path(__file__).resolve().parents[1] / "config/arm_tracking.json"
    if source.is_file():
        return source
    return Path(__file__).resolve().parents[2] / "share/chassis/config/arm_tracking.json"


def parser():
    result = recognition.parser()
    result.description = ("Center markers or record a 3D goal and approach it with the arm. "
                          "Preview by default; joystick mode waits for button6.")
    result.set_defaults(marker_type="aruco", print_hz=1.0)
    result.add_argument("--tracking-config", type=Path, default=default_config())
    result.add_argument("--tracking-mode", choices=("pixel", "pixel-depth", "depth", "approach"), default="pixel",
                        help="pixel: center the image midpoint without depth (default); "
                             "pixel-depth: also hold the optical depth acquired at session start; "
                             "depth: track height 7.45 cm below the 3D midpoint; "
                             "approach: align sideways, record a goal with configured X clearance "
                             "(4 cm by default), then reach it")
    mode = result.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true",
                      help="enable configured arm tracking and chassis left/right motion; "
                           "default only displays targets")
    mode.add_argument("--joystick", action="store_true",
                      help="wait for joystick button6 to start/stop tracking; camera stays open")
    return result


def invalid_tracking(reason):
    return dict(valid=False, reason=reason, frame_id="arm_shoulder", target_m=None)


def run(args):
    import cv2
    from vision.arm_tracking import ArmTracker, load_config
    from vision.arm_approach import ArmApproach
    from vision.qr_midpoint import ArucoMidpointDetector, QrMidpointDetector, annotate
    from vision.gemini_camera import GeminiCamera

    raw_config = json.loads(args.tracking_config.read_text())
    config = load_config(args.tracking_config) if raw_config.get("calibrated") is True else None
    control_capable = args.execute or args.joystick
    pixel_tracking = args.tracking_mode in ("pixel", "pixel-depth")
    if config is None and control_capable:
        raise ValueError("camera extrinsics are not calibrated; complete tracking-config before control")
    def make_tracker(capture_after=-math.inf):
        if not config:
            return None
        if args.tracking_mode == "approach":
            return ArmApproach(config, capture_after=capture_after)
        return ArmTracker(config, mode=args.tracking_mode)

    tracker = make_tracker()
    detector = (ArucoMidpointDetector(args.aruco_dictionary or "DICT_4X4_50", args.marker_ids or (0, 1))
                if args.marker_type == "aruco" else QrMidpointDetector(args.qr_texts or ()))
    display = not args.headless and bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    camera = ros = output = None
    count, last_print = 0, -math.inf
    clock_anchor = None
    source = "image" if args.image else "gemini335"
    control_requested = args.execute
    enable_error = None
    disable_pending = False
    last_disable_attempt = -math.inf

    def disable_control():
        nonlocal disable_pending, last_disable_attempt, enable_error
        last_disable_attempt = time.monotonic()
        try:
            ros.disable()
            disable_pending = False
            enable_error = None
        except RuntimeError as error:
            # No more targets are sent. Keep the camera open while retrying handover.
            disable_pending = True
            enable_error = str(error)
            ros.require_joystick_release()
            print("Tracking stop awaiting controller: " + enable_error, file=sys.stderr)

    def refresh_request(now):
        nonlocal control_requested, tracker, enable_error
        if not args.joystick:
            return
        requested = ros.joystick_requested(now)
        if ros.take_joystick_cancel():
            # Preserve an OFF received during a service wait, even if ON followed it.
            if requested:
                ros.require_joystick_release()
            requested = False
        if disable_pending:
            requested = False
            if now - last_disable_attempt >= 0.5:
                disable_control()
            ros.require_joystick_release()
        if requested != control_requested:
            control_requested = requested
            if not requested:
                disable_control()
            # Every new button session must acquire new consecutive frames.
            tracker = make_tracker(capture_after=now)
            if requested:
                enable_error = None

    def emit(record, force=False):
        nonlocal last_print
        text = json.dumps(record, ensure_ascii=False, allow_nan=False)
        if output is not None:
            output.write(text + "\n")
            output.flush()
        now = time.monotonic()
        if force or now - last_print >= 1.0 / args.print_hz:
            print(text, flush=True)
            last_print = now

    try:
        if args.json_output:
            output = args.json_output.open("w", encoding="utf-8")
        if not args.image:
            from vision.arm_tracking_ros import TrackingRosLink
            try:
                ros = TrackingRosLink(execute=control_capable, joystick=args.joystick,
                                      track_x=args.tracking_mode in ("pixel-depth", "approach"),
                                      goal_mode=args.tracking_mode == "approach")
            except ImportError:
                if control_capable:
                    raise RuntimeError("ROS Python environment unavailable; use start_arm_tracking.sh")
                print("ROS feedback unavailable; displaying vision only.", file=sys.stderr)
            # Gemini 335 minimum depth is about 14 cm at 640x360, versus 18 cm
            # at the SDK's usual 848x480 default. Keep RGB geometry independent.
            depth_width, depth_height = (640, 360) if args.tracking_mode == "approach" else (0, 0)
            camera = GeminiCamera(serial=args.serial,
                                  color_only=args.tracking_mode == "pixel" or args.color_only,
                                  width=args.width, height=args.height, fps=args.fps,
                                  timeout_ms=args.timeout_ms,
                                  depth_width=depth_width, depth_height=depth_height)
            print("Gemini connected: " + json.dumps(camera.start()), file=sys.stderr)
        while not args.max_frames or count < args.max_frames:
            if ros:
                ros.spin()
                refresh_request(time.monotonic())
            if args.image:
                frame = None
                image = cv2.imread(str(args.image))
                if image is None:
                    raise ValueError(f"cannot read image: {args.image}")
                received = time.monotonic()
                stamp = None
            else:
                frame = camera.read()
                received = time.monotonic()
                device_s = frame["timestamp_ms"] / 1000.0
                if clock_anchor is None:
                    clock_anchor = (received, device_s)
                # Do not relabel a delayed SDK frame as newly captured.
                captured = min(received, clock_anchor[0] + device_s - clock_anchor[1])
                stamp = ros.capture_stamp(received - captured) if ros else None
                received = captured
                image = frame["color_bgr"]
            if args.tracking_mode == "pixel":
                # No depth lookup is performed, even if a caller supplies a depth frame.
                detection = recognition.process_image(detector, image, source)
                detection["device_timestamp_ms"] = frame["timestamp_ms"] if frame else None
                detection["position_3d"] = recognition.unavailable_position("not_used_in_pixel_mode")
            else:
                detection = recognition.process_image(detector, image, source, frame)
            if ros:
                ros.spin()
            now = time.monotonic()
            refresh_request(now)
            if ros:
                pose, pose_at = ros.fresh_pose(now)
            else:
                pose = pose_at = None
            tracking = (tracker.evaluate(detection, now, received, pose, pose_at)
                        if tracker else invalid_tracking("calibration_required"))
            if control_requested and pose is None:
                tracking = tracker.invalidate("arm_feedback_missing_or_stale")
            if control_requested:
                if tracking["valid"]:
                    if not ros.enabled:
                        try:
                            ros.enable()
                            # A cancel received while waiting for the service wins.
                            refresh_request(time.monotonic())
                            if args.tracking_mode == "approach" and control_requested:
                                # Frozen goals must come from a capture after the controller's hold.
                                tracker = make_tracker(capture_after=time.monotonic())
                            # Enable/hold happened after this frame was captured.
                            tracking = invalid_tracking("arming_wait_for_new_frame")
                        except RuntimeError as error:
                            if not args.joystick:
                                raise
                            control_requested = False
                            ros.require_joystick_release()
                            disable_control()
                            tracker = make_tracker(capture_after=time.monotonic())
                            if not disable_pending:
                                enable_error = str(error)
                            print("Tracking not started: " + enable_error, file=sys.stderr)
                    else:
                        if args.tracking_mode == "approach" and tracking.get("command_kind") == "goal":
                            ros.publish_goal(tracking["target_m"])
                        else:
                            ros.publish(tracking["target_m"], stamp)
                else:
                    ros.stop()
            if args.joystick and not control_requested:
                tracking = invalid_tracking("disable_pending" if disable_pending else
                                            "enable_failed" if enable_error else "button6_to_start")
                if enable_error:
                    tracking["detail"] = enable_error
            tracking["mode"] = ("execute" if control_requested else
                                "standby" if args.joystick else "preview")
            tracking["tracking_mode"] = args.tracking_mode
            detection["tracking"] = tracking
            emit(detection, force=bool(args.image))
            count += 1
            if display or args.output:
                rendered = (annotate(image, detection) if pixel_tracking
                            else recognition.draw(image, detection))
                if pixel_tracking:
                    center_x = round((image.shape[1] - 1) / 2.)
                    center_y = round((image.shape[0] - 1) / 2.)
                    cv2.line(rendered, (0, center_y), (image.shape[1] - 1, center_y),
                             (255, 200, 0), 1, cv2.LINE_AA)
                    cv2.line(rendered, (center_x, 0), (center_x, image.shape[0] - 1),
                             (255, 200, 0), 1, cv2.LINE_AA)
                status = ((f"dx={tracking['pixel_error_x']:+.1f} "
                           f"dy={tracking['pixel_error_y']:+.1f} px")
                          if tracking["valid"] and pixel_tracking else
                          (f"{tracking['phase']} x={tracking['target_m'][0]:.3f} "
                           f"z={tracking['target_m'][2]:.3f} m")
                          if tracking["valid"] and args.tracking_mode == "approach" else
                          (f"y={tracking['target_m'][1]:+.3f} "
                           f"z={tracking['target_m'][2]:.3f} m")
                          if tracking["valid"] else tracking["reason"])
                label = ("TRACK" if control_requested else
                         "READY" if args.joystick else "PREVIEW")
                label += {"pixel": " PIXEL 2D: ", "pixel-depth": " PIXEL+DEPTH: ",
                          "depth": " DEPTH 3D: ", "approach": " APPROACH: "}[args.tracking_mode] + status
                cv2.putText(rendered, label, (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 200, 255), 1, cv2.LINE_AA)
                detail = (f"depth={tracking['depth_m']:.3f} m  "
                          f"hold={tracking['depth_reference_m']:.3f} m"
                          if args.tracking_mode == "pixel-depth" and tracking["valid"] else
                          "chassis left/right + arm forward/back/up/down"
                          if args.tracking_mode == "pixel-depth" else
                          "align -> record goal -> reach -> hold"
                          if args.tracking_mode == "approach" else
                          "chassis left/right + arm up/down")
                cv2.putText(rendered, detail, (10, 72),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1, cv2.LINE_AA)
                if args.joystick:
                    cv2.putText(rendered, "button6: start/stop | q/Esc: quit", (10, 94),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1, cv2.LINE_AA)
                if args.output and not cv2.imwrite(str(args.output), rendered):
                    raise OSError(f"cannot write image: {args.output}")
                if display:
                    cv2.imshow("Arm and chassis tracking", rendered)
                    key = cv2.waitKey(0 if args.image else 1) & 0xff
                    if key in (27, ord("q")) or cv2.getWindowProperty("Arm and chassis tracking", cv2.WND_PROP_VISIBLE) < 1:
                        break
            if args.image:
                break
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        # Stop the control stream before waiting for camera shutdown or GUI cleanup.
        try:
            if ros:
                ros.close()
        finally:
            try:
                if camera:
                    camera.stop()
            finally:
                record = recognition.failure_record(source, "stopped", "tracking session ended")
                record["tracking"] = invalid_tracking("stopped")
                record["tracking"]["tracking_mode"] = args.tracking_mode
                try:
                    emit(record, force=True)
                finally:
                    if output:
                        output.close()
                    if display:
                        cv2.destroyAllWindows()


def main(argv=None):
    arguments = parser()
    args = arguments.parse_args(argv)
    recognition.validate_options(args, arguments)
    if (args.execute or args.joystick) and args.image:
        arguments.error("--execute/--joystick requires a live camera")
    if (args.execute or args.joystick) and args.tracking_mode != "pixel" and args.color_only:
        arguments.error("depth control requires live color and depth")
    if not args.tracking_config.is_file():
        arguments.error(f"tracking config does not exist: {args.tracking_config}")
    for path in (args.output, args.json_output):
        if path and (path.resolve() == args.tracking_config.resolve()
                     or (path.exists() and path.samefile(args.tracking_config))):
            arguments.error("output must not overwrite tracking calibration")
    try:
        return run(args)
    except Exception as error:
        print(f"Arm tracking failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
