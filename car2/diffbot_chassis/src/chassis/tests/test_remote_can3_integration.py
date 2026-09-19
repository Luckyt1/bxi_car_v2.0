#!/usr/bin/env python3

import argparse
import os
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import uuid

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rcl_interfaces.msg import ParameterType, ParameterValue, SetParametersResult
    from rcl_interfaces.srv import GetParameters, SetParametersAtomically
except ImportError as error:
    rclpy = None
    _RCLPY_IMPORT_ERROR = error
else:
    _RCLPY_IMPORT_ERROR = None


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


def wait_until(predicate, timeout=3.0, interval=0.02):
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        try:
            if predicate():
                return True
        except AssertionError as error:
            last_error = error
        time.sleep(interval)
    if last_error is not None:
        raise last_error
    return False


class FakeCan3Server:
    def __init__(self, context, node_name, cmd_topic):
        self.context = context
        self.node = rclpy.create_node(f"fake_can3_{uuid.uuid4().hex[:8]}", context=context)
        self.values = {
            "can3_motor_1_angle_deg": 0.0,
            "can3_motor_2_angle_deg": 0.0,
            "can3_motor_3_angle_deg": 0.0,
        }
        self.get_delay = 0.0
        self.set_success = True
        self.get_calls = []
        self.set_calls = []
        self.twists = []
        self.lock = threading.Lock()
        self.get_service = self.node.create_service(
            GetParameters, f"{node_name}/get_parameters", self._handle_get
        )
        self.set_service = self.node.create_service(
            SetParametersAtomically,
            f"{node_name}/set_parameters_atomically",
            self._handle_set,
        )
        self.subscription = self.node.create_subscription(Twist, cmd_topic, self._on_twist, 10)
        self.executor = rclpy.executors.MultiThreadedExecutor(num_threads=2, context=context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _handle_get(self, request, response):
        if self.get_delay > 0.0:
            time.sleep(self.get_delay)
        with self.lock:
            self.get_calls.extend(request.names)
            for name in request.names:
                value = ParameterValue()
                if name in self.values:
                    value.type = ParameterType.PARAMETER_DOUBLE
                    value.double_value = float(self.values[name])
                else:
                    value.type = ParameterType.PARAMETER_NOT_SET
                response.values.append(value)
        return response

    def _handle_set(self, request, response):
        result = SetParametersResult()
        result.successful = self.set_success
        result.reason = "" if self.set_success else "injected failure"
        with self.lock:
            for parameter in request.parameters:
                value = parameter.value.double_value
                self.set_calls.append((parameter.name, value))
                if self.set_success:
                    self.values[parameter.name] = value
        response.result = result
        return response

    def _on_twist(self, message):
        with self.lock:
            self.twists.append(message)

    def snapshot(self):
        with self.lock:
            return list(self.get_calls), list(self.set_calls), dict(self.values), list(self.twists)

    def close(self):
        self.executor.shutdown()
        self.thread.join(timeout=1.0)
        self.node.destroy_node()


class KeyControlHarness:
    def __init__(self, binary):
        if rclpy is None:
            raise unittest.SkipTest(f"rclpy is unavailable: {_RCLPY_IMPORT_ERROR}")
        self.binary = os.path.abspath(binary)
        self.proc = None
        self.server = None
        self.context = None
        self.writer = None
        self.tmpdir = tempfile.mkdtemp(prefix="key-control-can3-")
        self.fifo = os.path.join(self.tmpdir, "js0")
        token = uuid.uuid4().hex[:10]
        self.can3_node = f"/offline_can3_{token}"
        self.cmd_topic = f"/offline_cmd_vel_{token}"
        self.env = os.environ.copy()
        self.env["ROS_DOMAIN_ID"] = str(120 + (os.getpid() % 80))
        self.env["ROS_LOCALHOST_ONLY"] = "1"
        self.env["ROS_LOG_DIR"] = os.path.join(self.tmpdir, "ros-log")
        self.env["RCUTILS_LOGGING_USE_STDOUT"] = "1"
        os.makedirs(self.env["ROS_LOG_DIR"], exist_ok=True)
        self.old_ros_env = {
            name: os.environ.get(name) for name in self.env if name.startswith("ROS_")
        }
        try:
            os.environ.update(
                {
                    "ROS_DOMAIN_ID": self.env["ROS_DOMAIN_ID"],
                    "ROS_LOCALHOST_ONLY": self.env["ROS_LOCALHOST_ONLY"],
                    "ROS_LOG_DIR": self.env["ROS_LOG_DIR"],
                }
            )
            os.mkfifo(self.fifo)
            self.writer = os.open(self.fifo, os.O_RDWR | os.O_NONBLOCK)
            self.context = rclpy.Context()
            rclpy.init(context=self.context)
            self.server = FakeCan3Server(self.context, self.can3_node, self.cmd_topic)
            self.proc = subprocess.Popen(
                [
                    self.binary,
                    "--ros-args",
                    "-p",
                    "start_chassis:=false",
                    "-p",
                    f"device_path:={self.fifo}",
                    "-p",
                    f"cmd_vel_topic:={self.cmd_topic}",
                    "-p",
                    f"can3_node:={self.can3_node}",
                    "-p",
                    "axis_angular:=6",
                    "-p",
                    "timeout_sec:=0.0",
                    "-p",
                    "joystick_low_pass_time_constant_s:=0.0",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                env=self.env,
            )
            time.sleep(0.4)
            self._ensure_alive()
        except Exception:
            self.close()
            raise

    def _ensure_alive(self):
        if self.proc.poll() is not None:
            output = self.proc.stdout.read() if self.proc.stdout else ""
            raise AssertionError(f"key_control exited early with {self.proc.returncode}\n{output}")

    def close(self):
        if getattr(self, "proc", None) is not None and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=2.0)
        if getattr(self, "proc", None) is not None and self.proc.stdout is not None:
            self.proc.stdout.close()
        if getattr(self, "server", None) is not None:
            self.server.close()
        if getattr(self, "context", None) is not None:
            self.context.try_shutdown()
        if getattr(self, "writer", None) is not None:
            os.close(self.writer)
            self.writer = None
        for name, value in getattr(self, "old_ros_env", {}).items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def event(self, event_type, number, value, initial=False):
        flags = event_type | (JS_EVENT_INIT if initial else 0)
        payload = struct.pack(
            "IhBB", int(time.monotonic() * 1000) & 0xFFFFFFFF, value, flags, number
        )
        os.write(self.writer, payload)
        time.sleep(0.08)
        self._ensure_alive()

    def axis(self, number, value, initial=False):
        self.event(JS_EVENT_AXIS, number, value, initial)

    def button(self, number, pressed, initial=False):
        self.event(JS_EVENT_BUTTON, number, 1 if pressed else 0, initial=initial)

    def neutral_startup(self):
        self.button(1, False, initial=True)
        self.button(3, False, initial=True)
        self.axis(0, 0, initial=True)
        self.axis(3, 0, initial=True)
        self.axis(6, 0, initial=True)
        self.axis(7, 0, initial=True)
        time.sleep(0.2)

    def step_up(self):
        self.axis(7, -32767)

    def step_down(self):
        self.axis(7, 32767)

    def step_neutral(self):
        self.axis(7, 0)

    def select_next(self):
        self.button(1, True)

    def select_previous(self):
        self.button(3, True)

    def select_neutral(self):
        self.button(1, False)
        self.button(3, False)

    def wait_for_sets(self, count, timeout=3.0):
        ok = wait_until(lambda: len(self.server.snapshot()[1]) >= count, timeout=timeout)
        if not ok:
            _, sets, _, _ = self.server.snapshot()
            raise AssertionError(f"expected {count} set calls, got {sets}")

    def wait_for_yaw(self, predicate, timeout=3.0):
        def has_matching_yaw():
            _, _, _, twists = self.server.snapshot()
            return any(predicate(message.angular.z) for message in twists)

        ok = wait_until(has_matching_yaw, timeout=timeout)
        if not ok:
            _, _, _, twists = self.server.snapshot()
            yaw_values = [message.angular.z for message in twists]
            raise AssertionError(f"expected matching yaw, got {yaw_values}")


class RemoteCan3IntegrationTest(unittest.TestCase):
    binary = None

    def setUp(self):
        self.harness = KeyControlHarness(self.binary)

    def tearDown(self):
        self.harness.close()

    def test_up_down_steps_once_per_neutral_return(self):
        h = self.harness
        h.neutral_startup()

        h.step_up()
        h.wait_for_sets(1)
        h.step_up()
        time.sleep(0.3)
        self.assertEqual(h.server.snapshot()[1], [("can3_motor_1_angle_deg", 1.0)])

        h.step_neutral()
        h.step_down()
        h.wait_for_sets(2)
        self.assertEqual(
            h.server.snapshot()[1],
            [("can3_motor_1_angle_deg", 1.0), ("can3_motor_1_angle_deg", 0.0)],
        )

    def test_buttons_select_motor_once_and_do_not_publish_yaw(self):
        h = self.harness
        h.neutral_startup()

        h.select_next()
        time.sleep(0.3)
        h.select_next()
        time.sleep(0.3)
        h.select_neutral()
        h.step_up()
        h.wait_for_sets(1)
        _, sets, _, twists = h.server.snapshot()
        self.assertEqual(sets, [("can3_motor_2_angle_deg", 1.0)])
        self.assertTrue(twists)
        self.assertTrue(all(abs(message.angular.z) < 1e-9 for message in twists))

        h.step_neutral()
        h.select_previous()
        time.sleep(0.3)
        h.select_previous()
        time.sleep(0.3)
        h.select_neutral()
        h.step_up()
        h.wait_for_sets(2)
        _, sets, _, _ = h.server.snapshot()
        self.assertEqual(
            sets,
            [("can3_motor_2_angle_deg", 1.0), ("can3_motor_1_angle_deg", 1.0)],
        )

    def test_axis6_controls_yaw_without_selecting_can3_motor(self):
        h = self.harness
        h.neutral_startup()

        h.axis(6, 32767)
        h.wait_for_yaw(lambda value: value < -0.05)
        h.axis(6, -32767)
        h.wait_for_yaw(lambda value: value > 0.05)
        h.axis(6, 0)
        h.step_up()
        h.wait_for_sets(1)
        self.assertEqual(h.server.snapshot()[1], [("can3_motor_1_angle_deg", 1.0)])

    def test_startup_held_step_axis_does_not_arm_until_neutral(self):
        h = self.harness
        h.button(1, False, initial=True)
        h.button(3, False, initial=True)
        h.axis(0, 0, initial=True)
        h.axis(3, 0, initial=True)
        h.axis(6, 0, initial=True)
        h.axis(7, -32767, initial=True)
        time.sleep(0.3)
        h.step_up()
        time.sleep(0.3)
        self.assertEqual(h.server.snapshot()[1], [])

        h.step_neutral()
        h.step_up()
        h.wait_for_sets(1)
        self.assertEqual(h.server.snapshot()[1], [("can3_motor_1_angle_deg", 1.0)])

    def test_startup_held_selection_button_waits_for_release_without_ghost_selection(self):
        h = self.harness
        h.button(1, True, initial=True)
        h.button(3, False, initial=True)
        h.axis(0, 0, initial=True)
        h.axis(3, 0, initial=True)
        h.axis(6, 0, initial=True)
        h.axis(7, 0, initial=True)
        time.sleep(0.3)
        h.step_up()
        time.sleep(0.3)
        self.assertEqual(h.server.snapshot()[1], [])

        h.button(1, False)
        time.sleep(0.4)
        h.step_neutral()
        h.step_up()
        h.wait_for_sets(1)
        self.assertEqual(h.server.snapshot()[1], [("can3_motor_1_angle_deg", 1.0)])

    def test_stop_button_blocks_new_steps(self):
        h = self.harness
        h.neutral_startup()
        h.button(0, True)
        h.step_up()
        time.sleep(0.4)
        self.assertEqual(h.server.snapshot()[1], [])

    def test_delayed_get_cancelled_by_stop_or_selection_change_never_sets_old_target(self):
        h = self.harness
        h.neutral_startup()
        h.server.get_delay = 0.6

        h.step_up()
        time.sleep(0.15)
        h.button(0, True)
        time.sleep(0.8)
        self.assertEqual(h.server.snapshot()[1], [])

        h.button(0, False)
        h.step_neutral()
        time.sleep(0.2)
        h.server.get_delay = 0.6
        h.step_up()
        time.sleep(0.15)
        h.select_next()
        time.sleep(0.8)
        self.assertEqual(h.server.snapshot()[1], [])

    def test_rejected_set_does_not_update_fake_target(self):
        h = self.harness
        h.neutral_startup()
        h.server.set_success = False
        h.step_up()
        h.wait_for_sets(1)
        _, sets, values, _ = h.server.snapshot()
        self.assertEqual(sets, [("can3_motor_1_angle_deg", 1.0)])
        self.assertEqual(values["can3_motor_1_angle_deg"], 0.0)

    def test_timed_out_get_response_does_not_late_set(self):
        h = self.harness
        h.neutral_startup()
        h.server.get_delay = 2.6
        h.step_up()
        time.sleep(3.0)
        self.assertEqual(h.server.snapshot()[1], [])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("key_control", help="path to the key_control executable")
    args = parser.parse_args()
    RemoteCan3IntegrationTest.binary = args.key_control
    if not os.path.exists(args.key_control):
        raise SystemExit(f"key_control executable not found: {args.key_control}")
    unittest.main(argv=[sys.argv[0]], verbosity=2)


if __name__ == "__main__":
    main()
