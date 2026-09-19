"""Offline tests for the CAN3 control terminal."""

import curses
import contextlib
import io
import importlib.util
from pathlib import Path
import subprocess
import sys
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "can3_control.py"


def load_module():
    """Load the script as a plain Python module without ROS."""
    spec = importlib.util.spec_from_file_location("can3_control", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


can3_control = load_module()


class Future:
    """Minimal future used by the mock parameter client."""

    def __init__(self, result=None, done=False):
        self._result = result
        self._done = done

    def done(self):
        """Return whether the future is complete."""
        return self._done

    def result(self):
        """Return the stored result."""
        return self._result

    def complete(self, result=None):
        """Mark the future complete."""
        if result is not None:
            self._result = result
        self._done = True


class MockClient:
    """Mock of the script's parameter-client adapter."""

    def __init__(self):
        self.ready = True
        self.get_futures = []
        self.set_futures = []
        self.get_calls = []
        self.set_calls = []
        self.removed = []
        self.values = {}
        self.set_success = True
        self.set_reason = ""
        self.raise_on_get = None
        self.raise_on_set = None

    def services_ready(self):
        """Return configured readiness."""
        return self.ready

    def get_parameter(self, name):
        """Record and return a pending get future."""
        if self.raise_on_get is not None:
            raise self.raise_on_get
        self.get_calls.append(name)
        future = Future()
        self.get_futures.append(future)
        return future

    def get_parameter_result(self, future, name):
        """Return a configured value."""
        return self.values[name]

    def set_parameter(self, name, value):
        """Record and return a pending set future."""
        if self.raise_on_set is not None:
            raise self.raise_on_set
        self.set_calls.append((name, value))
        future = Future()
        self.set_futures.append(future)
        return future

    def set_parameter_result(self, future):
        """Return configured set result."""
        return self.set_success, self.set_reason

    def remove_pending_request(self, future):
        """Record cleanup of a pending request."""
        self.removed.append(future)


class NarrowScreen:
    """Curses-like screen that raises when writing in a narrow terminal."""

    def __init__(self):
        self.keys = []
        self.writes = []

    def erase(self):
        """Pretend to erase the screen."""

    def refresh(self):
        """Pretend to refresh the screen."""

    def getmaxyx(self):
        """Return a very narrow screen."""
        return (3, 2)

    def addnstr(self, row, col, line, width):
        """Raise like curses can do at narrow edges."""
        self.writes.append((row, col, line, width))
        raise curses.error("narrow")


class KeyScreen:
    """Curses-like screen with a finite key buffer."""

    def __init__(self, keys):
        self.keys = list(keys)

    def getch(self):
        """Return buffered keys then -1."""
        if self.keys:
            return self.keys.pop(0)
        return -1


class PendingClient:
    """Mock ROS service client that records pending-request cleanup."""

    def __init__(self):
        self.removed = []

    def remove_pending_request(self, future):
        """Record cleanup and return None like rclpy on missing futures."""
        self.removed.append(future)
        return None


class Can3ControlTest(unittest.TestCase):
    """Behavior tests for the non-ROS control logic."""

    def test_help_exits_without_importing_rclpy(self):
        result = subprocess.run(
            [sys.executable, str(SCRIPT), "--help"],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn("--node", result.stdout)
        self.assertEqual(result.stderr, "")

    def test_adjustment_reads_current_target_before_setting_one_degree_step(self):
        client = MockClient()
        client.values["can3_motor_2_angle_deg"] = 12.5
        model = can3_control.ControlModel(timeout_s=2.0)
        model.select_motor(2)

        self.assertTrue(model.begin_adjust(client, +1, now_s=10.0))
        self.assertEqual(client.get_calls, ["can3_motor_2_angle_deg"])
        self.assertEqual(client.set_calls, [])

        client.get_futures[0].complete()
        self.assertIsNone(model.poll(client, now_s=10.1))
        self.assertEqual(client.set_calls, [("can3_motor_2_angle_deg", 13.5)])

        client.set_futures[0].complete()
        result = model.poll(client, now_s=10.2)
        self.assertTrue(result.successful)
        self.assertEqual(model.confirmed_targets[2], 13.5)

    def test_down_key_success_sets_minus_one_degree(self):
        client = MockClient()
        client.values["can3_motor_1_angle_deg"] = 3.0
        model = can3_control.ControlModel(timeout_s=2.0)

        self.assertFalse(can3_control.handle_keys([curses.KEY_DOWN], model, client))
        client.get_futures[0].complete()
        self.assertIsNone(model.poll(client, now_s=1.0))
        self.assertEqual(client.set_calls, [("can3_motor_1_angle_deg", 2.0)])

        client.set_futures[0].complete()
        result = model.poll(client, now_s=1.1)
        self.assertTrue(result.successful)
        self.assertEqual(model.confirmed_targets[1], 2.0)

    def test_inflight_operation_blocks_key_pileup(self):
        client = MockClient()
        model = can3_control.ControlModel(timeout_s=2.0)

        self.assertTrue(model.begin_adjust(client, +1, now_s=0.0))
        self.assertFalse(
            can3_control.handle_keys(
                [curses.KEY_UP, curses.KEY_DOWN, curses.KEY_UP], model, client
            )
        )
        self.assertEqual(client.get_calls, ["can3_motor_1_angle_deg"])
        self.assertIn("在途", model.status)

    def test_drained_key_batch_processes_q_first_and_one_direction(self):
        client = MockClient()
        model = can3_control.ControlModel(timeout_s=2.0)

        self.assertEqual(
            can3_control.drain_keys(KeyScreen([curses.KEY_UP, curses.KEY_UP])),
            [curses.KEY_UP, curses.KEY_UP],
        )
        self.assertFalse(can3_control.handle_keys([curses.KEY_UP, curses.KEY_UP], model, client))
        self.assertEqual(client.get_calls, ["can3_motor_1_angle_deg"])
        self.assertTrue(can3_control.handle_keys([curses.KEY_UP, ord("q")], model, client))
        self.assertEqual(client.get_calls, ["can3_motor_1_angle_deg"])

    def test_key_batch_uses_event_order_for_selection_and_arrow(self):
        client = MockClient()
        model = can3_control.ControlModel(timeout_s=2.0)
        self.assertFalse(can3_control.handle_keys([curses.KEY_UP, ord("3")], model, client))
        self.assertEqual(client.get_calls, ["can3_motor_1_angle_deg"])
        self.assertEqual(model.selected_motor, 1)

        client = MockClient()
        model = can3_control.ControlModel(timeout_s=2.0)
        self.assertFalse(can3_control.handle_keys([ord("3"), curses.KEY_UP], model, client))
        self.assertEqual(client.get_calls, ["can3_motor_3_angle_deg"])
        self.assertEqual(model.selected_motor, 3)

    def test_timeout_marks_result_unknown_and_next_attempt_regets_current_target(self):
        client = MockClient()
        model = can3_control.ControlModel(timeout_s=1.0)

        self.assertTrue(model.begin_adjust(client, +1, now_s=5.0))
        result = model.poll(client, now_s=6.0)
        self.assertFalse(result.successful)
        self.assertTrue(result.unknown)
        self.assertIsNone(model.confirmed_targets[1])
        self.assertEqual(client.removed, [client.get_futures[0]])

        client.values["can3_motor_1_angle_deg"] = 3.0
        self.assertTrue(model.begin_adjust(client, -1, now_s=6.1))
        self.assertEqual(client.get_calls, ["can3_motor_1_angle_deg", "can3_motor_1_angle_deg"])
        client.get_futures[1].complete()
        self.assertIsNone(model.poll(client, now_s=6.2))
        self.assertEqual(client.set_calls, [("can3_motor_1_angle_deg", 2.0)])

    def test_timeout_after_set_submission_cleans_set_future_and_regets_next_time(self):
        client = MockClient()
        client.values["can3_motor_1_angle_deg"] = 1.0
        model = can3_control.ControlModel(timeout_s=1.0)

        self.assertTrue(model.begin_adjust(client, +1, now_s=10.0))
        client.get_futures[0].complete()
        self.assertIsNone(model.poll(client, now_s=10.1))
        self.assertEqual(client.set_calls, [("can3_motor_1_angle_deg", 2.0)])

        result = model.poll(client, now_s=11.0)
        self.assertFalse(result.successful)
        self.assertTrue(result.unknown)
        self.assertIn("可能已提交", result.message)
        self.assertEqual(client.removed, [client.set_futures[0]])

        client.values["can3_motor_1_angle_deg"] = 2.0
        self.assertTrue(model.begin_adjust(client, +1, now_s=11.1))
        self.assertEqual(len(client.get_calls), 2)

    def test_server_rejection_displays_reason_without_updating_confirmed_target(self):
        client = MockClient()
        client.values["can3_motor_1_angle_deg"] = 0.0
        client.set_success = False
        client.set_reason = "calibration not complete"
        model = can3_control.ControlModel(timeout_s=2.0)

        self.assertTrue(model.begin_adjust(client, +1, now_s=1.0))
        client.get_futures[0].complete()
        self.assertIsNone(model.poll(client, now_s=1.1))
        client.set_futures[0].complete()
        result = model.poll(client, now_s=1.2)
        self.assertFalse(result.successful)
        self.assertIn("calibration not complete", result.message)
        self.assertIsNone(model.confirmed_targets[1])

    def test_get_and_set_start_exceptions_are_reported(self):
        client = MockClient()
        client.raise_on_get = RuntimeError("get exploded")
        model = can3_control.ControlModel(timeout_s=2.0)
        self.assertFalse(model.begin_adjust(client, +1, now_s=1.0))
        self.assertIn("get exploded", model.status)

        client = MockClient()
        client.values["can3_motor_1_angle_deg"] = 0.0
        client.raise_on_set = RuntimeError("set exploded")
        model = can3_control.ControlModel(timeout_s=2.0)
        self.assertTrue(model.begin_adjust(client, +1, now_s=1.0))
        client.get_futures[0].complete()
        result = model.poll(client, now_s=1.1)
        self.assertFalse(result.successful)
        self.assertIn("set exploded", result.message)

    def test_disconnected_service_does_not_start_operation(self):
        client = MockClient()
        client.ready = False
        model = can3_control.ControlModel(timeout_s=2.0)

        self.assertFalse(model.begin_adjust(client, +1, now_s=1.0))
        self.assertEqual(client.get_calls, [])
        self.assertIn("断连", model.status)

    def test_invalid_timeout_is_rejected(self):
        for value in ("0", "-1", "nan", "inf"):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                can3_control.parse_args(["--timeout", value])

    def test_render_survives_narrow_curses_errors(self):
        model = can3_control.ControlModel(timeout_s=2.0)
        screen = NarrowScreen()
        can3_control.render(screen, model, "/chassis")
        self.assertGreater(len(screen.writes), 0)

    def test_ros_adapter_tries_both_clients_when_removing_pending_request(self):
        adapter = object.__new__(can3_control.RosParameterClient)
        adapter.get_client = PendingClient()
        adapter.set_client = PendingClient()
        future = Future()

        adapter.remove_pending_request(future)

        self.assertEqual(adapter.get_client.removed, [future])
        self.assertEqual(adapter.set_client.removed, [future])


if __name__ == "__main__":
    unittest.main()
