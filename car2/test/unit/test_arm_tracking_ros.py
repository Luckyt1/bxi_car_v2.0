#!/usr/bin/env python3
"""Exercise tracking transport against localhost ROS simulators, never hardware.

Run after sourcing /opt/ros/humble/setup.bash, using the vision virtualenv's
Python. This file forces localhost-only ROS domain 194 before importing rclpy.
"""

import importlib.util
import math
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest


_LOG_DIRECTORY = tempfile.TemporaryDirectory(prefix="arm-tracking-ros-test-")
os.environ.update(ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID="194", ROS_LOG_DIR=_LOG_DIRECTORY.name)

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rcl_interfaces.msg import ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "arm_tracking_ros", ROOT / "src/thirdparty/vision/arm_tracking_ros.py")
transport = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(transport)


class SimulatedChassis:
    """A parameter service and message recorder with no controller imports."""

    def __init__(self):
        self.enabled = False
        self.track_x = False
        self.goal_mode = False
        self.supports_x_control = True
        self.supports_goal_control = True
        self.parameter_updates = []
        self.events, self.targets, self.stops = [], [], []
        self.goals = []
        self.context = Context()
        rclpy.init(args=[], context=self.context)
        self.node = rclpy.create_node(
            "chassis", context=self.context, start_parameter_services=False)
        self.node.create_service(GetParameters, "/chassis/get_parameters", self._get)
        self.node.create_service(
            SetParametersAtomically, "/chassis/set_parameters_atomically", self._set)
        self.pose_publisher = self.node.create_publisher(
            PoseStamped, "/arm_pose", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.joystick_publisher = self.node.create_publisher(Bool, "/arm_tracking_requested", 1)
        self.node.create_subscription(
            PointStamped, "/arm_tracking_target", self.targets.append, 10)
        self.node.create_subscription(
            PointStamped, "/arm_tracking_goal_target", self.goals.append, 10)
        self.node.create_subscription(Empty, "/arm_tracking_stop", self.stops.append, 10)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _get(self, request, response):
        self.events.append(("get", self.enabled))
        values = {"arm_tracking_enabled": self.enabled}
        if self.supports_x_control:
            values["arm_tracking_x_enabled"] = self.track_x
        if self.supports_goal_control:
            values["arm_tracking_goal_enabled"] = self.goal_mode
        response.values = [ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=values[name])
                           if name in values else ParameterValue()
                           for name in request.names]
        return response

    def _set(self, request, response):
        parameters = {parameter.name: parameter.value for parameter in request.parameters}
        valid = (len(request.parameters) == 3
                 and set(parameters) == {"arm_tracking_enabled", "arm_tracking_x_enabled",
                                         "arm_tracking_goal_enabled"}
                 and all(value.type == ParameterType.PARAMETER_BOOL for value in parameters.values()))
        response.result = SetParametersResult(successful=valid, reason="" if valid else "bad request")
        if valid:
            self.enabled = parameters["arm_tracking_enabled"].bool_value
            self.track_x = parameters["arm_tracking_x_enabled"].bool_value
            self.goal_mode = parameters["arm_tracking_goal_enabled"].bool_value
            self.parameter_updates.append({name: value.bool_value for name, value in parameters.items()})
            self.events.append(("set", self.enabled))
        return response

    def close(self):
        self.executor.shutdown(timeout_sec=2.)
        self.thread.join(timeout=2.)
        self.node.destroy_node()
        if self.context.ok():
            rclpy.shutdown(context=self.context)


class TrackingRosTests(unittest.TestCase):
    def setUp(self):
        self.sim = SimulatedChassis()
        self.addCleanup(self.sim.close)
        self.links = []
        self.addCleanup(self.close_links)

    def close_links(self):
        for link in self.links:
            link.close()

    def link(self, execute=False, joystick=False, track_x=False, goal_mode=False):
        result = transport.TrackingRosLink(execute=execute, joystick=joystick,
                                          track_x=track_x, goal_mode=goal_mode)
        self.links.append(result)
        return result

    def close_link(self, link):
        self.links.remove(link)
        link.close()

    def until(self, predicate, link=None, timeout=3.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if link is not None:
                link.spin()
            if predicate():
                return
            time.sleep(.002)
        self.fail("localhost simulated ROS operation did not complete before timeout")

    def pose_message(self, pitch=.6, age=0.):
        message = PoseStamped()
        message.header.frame_id = "arm_shoulder"
        ns = self.sim.node.get_clock().now().nanoseconds - int(age * 1e9)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, 1000000000)
        message.pose.position.x, message.pose.position.z = .35, .5
        message.pose.orientation.y = -math.sin(pitch / 2.)
        message.pose.orientation.w = math.cos(pitch / 2.)
        return message

    def send_joystick(self, link, requested):
        self.until(lambda: self.sim.joystick_publisher.get_subscription_count() > 0, link)
        previous = link.joystick_received_at
        self.sim.joystick_publisher.publish(Bool(data=requested))
        self.until(lambda: link.joystick_received_at != previous, link)

    def test_joystick_restart_requires_false_before_true_and_never_enables_itself(self):
        link = self.link(execute=True, joystick=True)
        self.send_joystick(link, True)
        self.assertFalse(link.joystick_requested(time.monotonic()))
        self.send_joystick(link, False)
        self.send_joystick(link, True)
        self.assertTrue(link.joystick_requested(time.monotonic()))
        self.assertEqual(self.sim.events, [])
        self.assertFalse(link.enabled)

    def test_joystick_timeout_and_stream_gap_require_a_new_false_true_sequence(self):
        link = self.link(execute=True, joystick=True)
        self.send_joystick(link, False)
        self.send_joystick(link, True)
        self.assertTrue(link.joystick_requested(time.monotonic()))

        self.assertFalse(link.joystick_requested(link.joystick_received_at + .31))
        self.send_joystick(link, True)
        self.assertFalse(link.joystick_requested(time.monotonic()))
        self.send_joystick(link, False)
        self.send_joystick(link, True)
        self.assertTrue(link.joystick_requested(time.monotonic()))
        # A callback after a long gap must disarm even before the next poll.
        link.joystick_received_at -= .31
        self.send_joystick(link, True)
        self.assertFalse(link.joystick_requested(time.monotonic()))
        self.send_joystick(link, False)
        self.send_joystick(link, True)
        self.assertTrue(link.joystick_requested(time.monotonic()))

    def test_intermediate_cancel_is_retained_when_another_on_follows(self):
        link = self.link(execute=True, joystick=True)
        self.send_joystick(link, False)
        self.send_joystick(link, True)
        self.assertFalse(link.take_joystick_cancel())
        self.send_joystick(link, False)
        self.send_joystick(link, True)
        self.assertTrue(link.joystick_requested(time.monotonic()))
        self.assertTrue(link.take_joystick_cancel())
        self.assertFalse(link.take_joystick_cancel())

    def test_disable_retains_transport_for_another_button_session(self):
        link = self.link(execute=True, joystick=True)
        link.disable()
        self.assertEqual(self.sim.events, [])
        link.enable()
        link.disable()
        self.assertFalse(self.sim.enabled)
        self.assertFalse(link.enabled)
        self.assertTrue(link.context.ok())
        link.enable()
        self.assertTrue(self.sim.enabled)
        self.close_link(link)
        self.assertFalse(self.sim.enabled)

    def send_pose(self, link, message=None, valid=True):
        self.until(lambda: self.sim.pose_publisher.get_subscription_count() > 0, link)
        if message is None:
            message = self.pose_message()
        if not valid:
            self.assertIsNotNone(link.pose, "seed a valid pose before checking invalidation")
        stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        next_publish = 0.

        def observed():
            nonlocal next_publish
            matched = ((link.pose is not None and link.pose_stamp_ns == stamp)
                       if valid else link.pose is None)
            if matched:
                return True
            # Graph discovery can precede the BEST_EFFORT writer handshake.
            # Spin before each retry to process an already delivered frame.
            if time.monotonic() >= next_publish:
                self.sim.pose_publisher.publish(message)
                next_publish = time.monotonic() + .03
            return False

        self.until(observed, link)
        return message

    def test_preview_receives_with_independent_context_and_creates_no_control_resources(self):
        link = self.link()
        self.assertIsNot(link.context, self.sim.context)
        self.assertIs(link.node.context, link.context)
        self.assertIs(link.executor.context, link.context)
        self.assertEqual(list(link.node.clients), [])
        topics = [publisher.topic_name for publisher in link.node.publishers]
        self.assertNotIn("/arm_tracking_target", topics)
        self.assertNotIn("/arm_tracking_stop", topics)
        for attribute in ("get_client", "set_client", "target_publisher", "stop_publisher"):
            self.assertFalse(hasattr(link, attribute), attribute)
        self.send_pose(link)
        pose, received = link.fresh_pose(time.monotonic())
        self.assertIsNotNone(received)
        for actual, expected in zip(pose, [.35, .5, .6]):
            self.assertAlmostEqual(actual, expected)
        pose[0] = 99.
        self.assertAlmostEqual(link.pose[0], .35)
        with self.assertRaisesRegex(RuntimeError, "preview cannot enable"):
            link.enable()
        with self.assertRaisesRegex(RuntimeError, "not enabled"):
            link.publish([.1, 0., .2], link.capture_stamp())
        link.stop(force=True)
        self.close_link(link)
        self.assertFalse(link.context.ok())
        self.assertTrue(self.sim.context.ok())
        self.assertFalse(self.sim.enabled)
        self.assertEqual(self.sim.events, [])
        self.assertEqual(self.sim.targets, [])
        self.assertEqual(self.sim.stops, [])

    def test_execute_reads_disabled_mode_then_enables_publishes_stops_and_disables(self):
        link = self.link(execute=True)
        with self.assertRaisesRegex(RuntimeError, "not enabled"):
            link.publish([.3, .02, .4], link.capture_stamp())
        link.enable()
        self.assertEqual(self.sim.events, [("get", False), ("set", True)])
        self.assertTrue(self.sim.enabled)
        self.assertTrue(link.enabled)
        self.assertTrue(link.enable_requested)
        self.until(lambda: link.target_publisher.get_subscription_count() > 0
                   and link.stop_publisher.get_subscription_count() > 0, link)
        stamp = link.capture_stamp(age_s=.02)
        link.publish([.3, .02, .4], stamp)
        self.until(lambda: len(self.sim.targets) == 1, link)
        message = self.sim.targets[0]
        self.assertEqual(message.header.frame_id, "arm_shoulder")
        self.assertEqual(message.header.stamp, stamp)
        self.assertEqual([message.point.x, message.point.y, message.point.z], [.3, .02, .4])
        self.assertTrue(link.sending)
        link.stop()
        self.until(lambda: len(self.sim.stops) == 1, link)
        self.assertFalse(link.sending)
        self.close_link(link)
        self.until(lambda: len(self.sim.stops) == 2)
        self.assertFalse(self.sim.enabled)
        self.assertFalse(link.enabled)
        self.assertFalse(link.enable_requested)
        self.assertEqual(self.sim.events, [("get", False), ("set", True), ("set", False)])

    def test_already_enabled_mode_is_not_taken_over_or_disabled_on_close(self):
        self.sim.enabled = True
        link = self.link(execute=True)
        with self.assertRaisesRegex(RuntimeError, "already enabled"):
            link.enable()
        self.assertFalse(link.enabled)
        self.assertFalse(link.enable_requested)
        self.close_link(link)
        self.assertTrue(self.sim.enabled)
        self.assertEqual(self.sim.events, [("get", True)])
        self.assertEqual(self.sim.targets, [])
        self.assertEqual(self.sim.stops, [])

    def test_x_control_is_enabled_atomically_and_cleared_on_exit(self):
        link = self.link(execute=True, track_x=True)
        link.enable()
        self.assertEqual(self.sim.parameter_updates, [
            {"arm_tracking_enabled": True, "arm_tracking_x_enabled": True,
             "arm_tracking_goal_enabled": False}])
        link.disable()
        self.assertEqual(self.sim.parameter_updates[-1],
                         {"arm_tracking_enabled": False, "arm_tracking_x_enabled": False,
                          "arm_tracking_goal_enabled": False})
        self.assertFalse(self.sim.track_x)

    def test_height_only_session_clears_a_previous_x_selection(self):
        self.sim.track_x = True
        link = self.link(execute=True)
        link.enable()
        self.assertTrue(self.sim.enabled)
        self.assertFalse(self.sim.track_x)
        self.assertEqual(self.sim.parameter_updates, [
            {"arm_tracking_enabled": True, "arm_tracking_x_enabled": False,
             "arm_tracking_goal_enabled": False}])

    def test_missing_controller_x_support_fails_before_enabling(self):
        self.sim.supports_x_control = False
        link = self.link(execute=True, track_x=True)
        with self.assertRaisesRegex(RuntimeError, "load the new build"):
            link.enable()
        self.assertFalse(link.enable_requested)
        self.assertFalse(self.sim.enabled)
        self.assertEqual(self.sim.parameter_updates, [])

    def test_recorded_goal_has_its_own_topic_and_current_command_time(self):
        link = self.link(execute=True, track_x=True, goal_mode=True)
        link.enable()
        self.assertEqual(self.sim.parameter_updates, [
            {"arm_tracking_enabled": True, "arm_tracking_x_enabled": True,
             "arm_tracking_goal_enabled": True}])
        self.until(lambda: link.goal_publisher.get_subscription_count() > 0, link)
        before = link.node.get_clock().now().nanoseconds
        link.publish_goal([.3, 0., .2])
        self.until(lambda: len(self.sim.goals) == 1, link)
        goal = self.sim.goals[0]
        self.assertEqual(goal.header.frame_id, "arm_shoulder")
        self.assertEqual([goal.point.x, goal.point.y, goal.point.z], [.3, 0., .2])
        issued = goal.header.stamp.sec * 1000000000 + goal.header.stamp.nanosec
        self.assertGreaterEqual(issued, before)
        self.assertLessEqual(issued, link.node.get_clock().now().nanoseconds)
        self.assertEqual(self.sim.targets, [])
        link.disable()
        self.assertFalse(self.sim.enabled)
        self.assertFalse(self.sim.goal_mode)
        with self.assertRaisesRegex(RuntimeError, "not enabled"):
            link.publish_goal([.3, 0., .2])

    def test_goal_commands_require_explicit_mode_and_controller_support(self):
        with self.assertRaisesRegex(ValueError, "requires X"):
            transport.TrackingRosLink(execute=True, goal_mode=True)
        regular = self.link(execute=True, track_x=True)
        regular.enable()
        with self.assertRaisesRegex(RuntimeError, "not enabled"):
            regular.publish_goal([.3, 0., .2])
        self.close_link(regular)
        self.sim.supports_goal_control = False
        goal_link = self.link(execute=True, track_x=True, goal_mode=True)
        with self.assertRaisesRegex(RuntimeError, "load the new build"):
            goal_link.enable()
        self.assertFalse(goal_link.enable_requested)
        self.assertFalse(self.sim.enabled)
        self.assertEqual(self.sim.goals, [])

    def test_pose_freshness_uses_message_age_and_rejects_expired_or_future_reads(self):
        link = self.link()
        self.send_pose(link, self.pose_message(age=.08))
        age = time.monotonic() - link.pose_received_at
        self.assertGreaterEqual(age, .08)
        self.assertLess(age, .25)
        received = link.pose_received_at
        self.assertIsNotNone(link.fresh_pose(received + .24)[0])
        self.assertEqual(link.fresh_pose(received + .26), (None, None))
        self.assertEqual(link.fresh_pose(received - .01), (None, None))
        self.assertEqual(link.fresh_pose(received + .11, max_age=.1), (None, None))

    def test_bad_pose_frame_geometry_quaternion_and_timestamps_clear_feedback(self):
        link = self.link()
        mutations = [
            ("frame", lambda m: setattr(m.header, "frame_id", "camera")),
            ("lateral", lambda m: setattr(m.pose.position, "y", .01)),
            ("nan_position", lambda m: setattr(m.pose.position, "x", math.nan)),
            ("infinite_position", lambda m: setattr(m.pose.position, "z", math.inf)),
            ("roll", lambda m: setattr(m.pose.orientation, "x", .01)),
            ("yaw", lambda m: setattr(m.pose.orientation, "z", .01)),
            ("nonunit", lambda m: setattr(m.pose.orientation, "w", .2)),
            ("nan_quaternion", lambda m: setattr(m.pose.orientation, "y", math.nan)),
            ("zero_stamp", lambda m: (setattr(m.header.stamp, "sec", 0),
                                      setattr(m.header.stamp, "nanosec", 0))),
        ]
        for name, mutate in mutations:
            with self.subTest(name=name):
                self.send_pose(link)
                message = self.pose_message()
                mutate(message)
                self.send_pose(link, message, valid=False)
                self.assertEqual(link.fresh_pose(time.monotonic()), (None, None))
        for age in (.5, -.5):
            with self.subTest(age=age):
                self.send_pose(link)
                self.send_pose(link, self.pose_message(age=age), valid=False)
                self.assertIsNone(link.pose_received_at)

    def test_duplicate_and_out_of_order_feedback_do_not_refresh_or_lower_high_watermark(self):
        link = self.link()
        earlier = self.send_pose(link)
        self.send_pose(link, earlier, valid=False)
        duplicate_stamp = earlier.header.stamp.sec * 1000000000 + earlier.header.stamp.nanosec
        self.assertEqual(link.pose_stamp_ns, duplicate_stamp)
        later = self.send_pose(link)
        later_stamp = later.header.stamp.sec * 1000000000 + later.header.stamp.nanosec
        self.assertGreater(later_stamp, duplicate_stamp)
        self.send_pose(link, earlier, valid=False)
        self.assertEqual(link.pose_stamp_ns, later_stamp)
        self.send_pose(link)
        self.assertIsNotNone(link.fresh_pose(time.monotonic())[0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
