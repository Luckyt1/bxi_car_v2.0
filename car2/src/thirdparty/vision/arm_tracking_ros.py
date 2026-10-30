"""ROS transport for the optional arm tracker; preview mode only subscribes."""

import math
import time


class TrackingRosLink:
    def __init__(self, execute=False, joystick=False, track_x=False, goal_mode=False):
        if joystick and not execute:
            raise ValueError("joystick mode requires control transport")
        if goal_mode and not track_x:
            raise ValueError("recorded-goal control requires X tracking")
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.context import Context
        from rclpy.duration import Duration
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        self.rclpy = rclpy
        self.context = Context()
        rclpy.init(args=[], context=self.context)
        self.node = rclpy.create_node("arm_visual_tracker", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.execute = execute
        self.track_x = track_x
        self.goal_mode = goal_mode
        self.enabled = False
        self.enable_requested = False
        self.joystick_received_at = None
        self.joystick_released = False
        self.joystick_request = False
        self.joystick_cancel_pending = False
        self.pose = self.pose_received_at = None
        self.pose_stamp_ns = None
        self.sending = False
        self.qos = QoSProfile(depth=1, lifespan=Duration(seconds=0.25))
        pose_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.node.create_subscription(
            PoseStamped, "/arm_pose", self._observe_pose, pose_qos)
        if execute:
            from geometry_msgs.msg import PointStamped
            from std_msgs.msg import Empty
            from rcl_interfaces.srv import GetParameters, SetParametersAtomically

            self.target_publisher = self.node.create_publisher(
                PointStamped, "/arm_tracking_target", self.qos)
            if goal_mode:
                self.goal_publisher = self.node.create_publisher(
                    PointStamped, "/arm_tracking_goal_target", self.qos)
            self.stop_publisher = self.node.create_publisher(
                Empty, "/arm_tracking_stop", self.qos)
            self.get_client = self.node.create_client(GetParameters, "/chassis/get_parameters")
            self.set_client = self.node.create_client(
                SetParametersAtomically, "/chassis/set_parameters_atomically")
        if joystick:
            from std_msgs.msg import Bool
            self.joystick_subscription = self.node.create_subscription(
                Bool, "/arm_tracking_requested", self._observe_joystick, self.qos)

    def _observe_joystick(self, message):
        now = time.monotonic()
        if (self.joystick_received_at is None
                or now - self.joystick_received_at > 0.3):
            self.require_joystick_release()
        self.joystick_received_at = now
        if not message.data:
            self.joystick_cancel_pending |= self.joystick_request
            self.joystick_released = True
            self.joystick_request = False
        elif self.joystick_released:
            self.joystick_request = True

    def require_joystick_release(self):
        # A restarted vision process or recovered stream cannot replay a held request.
        self.joystick_cancel_pending |= self.joystick_request
        self.joystick_received_at = None
        self.joystick_released = self.joystick_request = False

    def take_joystick_cancel(self):
        cancelled = self.joystick_cancel_pending
        self.joystick_cancel_pending = False
        return cancelled

    def joystick_requested(self, now):
        if (self.joystick_received_at is None
                or not 0 <= now - self.joystick_received_at <= 0.3):
            self.require_joystick_release()
            return False
        return self.joystick_request

    def _observe_pose(self, message):
        p, q = message.pose.position, message.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds - stamp) / 1e9
        if (message.header.frame_id != "arm_shoulder"
                or not all(math.isfinite(v) for v in values)
                or abs(p.y) > 1e-6 or abs(q.x) > 1e-6 or abs(q.z) > 1e-6
                or abs(q.y * q.y + q.w * q.w - 1.0) > 1e-6
                or stamp <= 0 or not 0 <= age <= 0.25
                or (self.pose_stamp_ns is not None and stamp <= self.pose_stamp_ns)):
            self.pose = self.pose_received_at = None
            return
        pitch = -2.0 * math.atan2(q.y, q.w)
        self.pose = [float(p.x), float(p.z), math.atan2(math.sin(pitch), math.cos(pitch))]
        self.pose_received_at = time.monotonic() - age
        self.pose_stamp_ns = stamp

    def spin(self):
        self.executor.spin_once(timeout_sec=0)

    def fresh_pose(self, now, max_age=0.25):
        if (self.pose is None or self.pose_received_at is None
                or not 0 <= now - self.pose_received_at <= max_age):
            return None, None
        return list(self.pose), self.pose_received_at

    def capture_stamp(self, age_s=0.0):
        now = self.node.get_clock().now()
        ns = now.nanoseconds - int(max(0.0, age_s) * 1e9)
        stamp = now.to_msg()
        stamp.sec, stamp.nanosec = divmod(ns, 1000000000)
        return stamp

    def _wait(self, future, timeout=2.0):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
        if not future.done():
            raise RuntimeError("arm tracking parameter service timed out")
        result = future.result()
        if result is None:
            raise RuntimeError("arm tracking parameter service returned no result")
        return result

    def _set_enabled(self, value):
        from rcl_interfaces.srv import SetParametersAtomically
        from rclpy.parameter import Parameter

        request = SetParametersAtomically.Request()
        request.parameters = [
            Parameter("arm_tracking_enabled", value=value).to_parameter_msg(),
            Parameter("arm_tracking_x_enabled", value=bool(value and self.track_x)).to_parameter_msg(),
            Parameter("arm_tracking_goal_enabled", value=bool(value and self.goal_mode)).to_parameter_msg(),
        ]
        response = self._wait(self.set_client.call_async(request))
        if not response.result.successful:
            raise RuntimeError("arm tracking mode rejected: " + response.result.reason)

    def enable(self):
        if not self.execute:
            raise RuntimeError("preview cannot enable motion")
        from rcl_interfaces.msg import ParameterType
        from rcl_interfaces.srv import GetParameters

        if not self.get_client.wait_for_service(timeout_sec=2.0) or not self.set_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("/chassis tracking services unavailable; controller must be running")
        request = GetParameters.Request()
        request.names = ["arm_tracking_enabled", "arm_tracking_x_enabled", "arm_tracking_goal_enabled"]
        response = self._wait(self.get_client.call_async(request))
        if (len(response.values) != 3
                or any(value.type != ParameterType.PARAMETER_BOOL for value in response.values)):
            raise RuntimeError("running controller does not provide recorded-goal control; load the new build first")
        if response.values[0].bool_value:
            raise RuntimeError("arm tracking is already enabled; refusing to take another session")
        # A response may be lost after acceptance. Cleanup must still send stop/disable.
        self.enable_requested = True
        self._set_enabled(True)
        self.enabled = True

    def publish(self, point, stamp):
        if not self.execute or not self.enabled:
            raise RuntimeError("arm tracking was not enabled")
        from geometry_msgs.msg import PointStamped

        message = PointStamped()
        message.header.frame_id = "arm_shoulder"
        message.header.stamp = stamp
        message.point.x, message.point.y, message.point.z = map(float, point)
        self.target_publisher.publish(message)
        self.sending = True

    def stop(self, force=False):
        if self.execute and self.enable_requested and (force or self.sending):
            from std_msgs.msg import Empty
            self.stop_publisher.publish(Empty())
        self.sending = False

    def publish_goal(self, point):
        if not self.execute or not self.enabled or not self.goal_mode:
            raise RuntimeError("recorded-goal control was not enabled")
        from geometry_msgs.msg import PointStamped

        message = PointStamped()
        message.header.frame_id = "arm_shoulder"
        # This topic timestamps a current execution command, not a new camera observation.
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.point.x, message.point.y, message.point.z = map(float, point)
        self.goal_publisher.publish(message)
        self.sending = True

    def disable(self):
        if self.execute and self.enable_requested:
            self.stop(force=True)
            self._set_enabled(False)
            self.enabled = self.enable_requested = False

    def close(self):
        try:
            self.disable()
        finally:
            self.executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
            if self.context.ok():
                self.rclpy.shutdown(context=self.context)
