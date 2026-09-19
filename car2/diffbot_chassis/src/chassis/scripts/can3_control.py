#!/usr/bin/env python3
"""Curses UI for adjusting CAN3 BXI motor angle targets through ROS parameters."""

from __future__ import annotations

import argparse
import curses
from dataclasses import dataclass
import math
import sys
import time
from typing import Optional


STEP_DEG = 1.0
SPEED_LIMIT_RPM = 60.0
KP = 200.0
KD = 4.0
ENCODING_RANGE_DEG = math.degrees(12.5)
PARAMETER_NAMES = {
    1: "can3_motor_1_angle_deg",
    2: "can3_motor_2_angle_deg",
    3: "can3_motor_3_angle_deg",
}


@dataclass
class OperationResult:
    """Result of one get-then-set target adjustment."""

    successful: bool
    message: str
    target_deg: Optional[float] = None
    unknown: bool = False


class GetThenSetOperation:
    """One in-flight operation that reads current target before setting a new target."""

    def __init__(self, client, motor_id: int, delta_deg: float, timeout_s: float, now_s: float):
        self.client = client
        self.motor_id = motor_id
        self.parameter_name = PARAMETER_NAMES[motor_id]
        self.delta_deg = delta_deg
        self.deadline_s = now_s + timeout_s
        self.get_future = client.get_parameter(self.parameter_name)
        self.set_future = None
        self.target_deg = None

    def poll(self, now_s: float) -> Optional[OperationResult]:
        """Advance the operation and return a result once it finishes or times out."""
        if now_s >= self.deadline_s:
            self.cancel_pending()
            if self.set_future is not None:
                return OperationResult(
                    False,
                    "结果未知：设置请求可能已提交并生效，无法撤回；下次操作会重新读取目标值",
                    unknown=True,
                )
            return OperationResult(
                False,
                "结果未知：读取参数服务超时，下次操作会重新读取目标值",
                unknown=True,
            )

        if self.set_future is None:
            if not self.get_future.done():
                return None
            try:
                current = float(
                    self.client.get_parameter_result(self.get_future, self.parameter_name)
                )
                self.target_deg = current + self.delta_deg
                self.set_future = self.client.set_parameter(self.parameter_name, self.target_deg)
            except Exception as exc:  # noqa: BLE001 - show service failure in the UI.
                return OperationResult(False, f"读取/提交失败：{exc}")
            return None

        if not self.set_future.done():
            return None
        try:
            successful, reason = self.client.set_parameter_result(self.set_future)
        except Exception as exc:  # noqa: BLE001 - show service failure in the UI.
            return OperationResult(False, f"设置失败：{exc}")
        if successful:
            return OperationResult(
                True,
                f"已确认 motor {self.motor_id} 目标 {self.target_deg:.1f} deg",
                self.target_deg,
            )
        return OperationResult(False, f"服务器拒绝：{reason or '未给出原因'}")

    def cancel_pending(self) -> None:
        """Remove the pending ROS request from the client when possible."""
        future = self.set_future if self.set_future is not None else self.get_future
        try:
            self.client.remove_pending_request(future)
        except AttributeError:
            pass


class ControlModel:
    """UI state and operation policy independent of curses and ROS."""

    def __init__(self, timeout_s: float = 2.0):
        self.selected_motor = 1
        self.confirmed_targets = {1: None, 2: None, 3: None}
        self.timeout_s = timeout_s
        self.operation = None
        self.status = "未连接：等待 /chassis 参数服务"
        self.connected = False

    def select_motor(self, motor_id: int) -> None:
        """Select one of the three CAN3 motors."""
        if motor_id in PARAMETER_NAMES:
            self.selected_motor = motor_id
            self.status = f"已选择 motor {motor_id}"

    def begin_adjust(self, client, direction: int, now_s: Optional[float] = None) -> bool:
        """Start a one-degree adjustment if no other operation is in flight."""
        if self.operation is not None:
            self.status = "已有一次参数操作在途，忽略方向键以防堆积"
            return False
        if not self._services_ready(client):
            self.connected = False
            self.status = "参数服务断连：无法发送调整"
            return False
        now = time.monotonic() if now_s is None else now_s
        self.connected = True
        delta = STEP_DEG if direction > 0 else -STEP_DEG
        try:
            self.operation = GetThenSetOperation(
                client, self.selected_motor, delta, self.timeout_s, now
            )
        except Exception as exc:  # noqa: BLE001 - show service failure in the UI.
            self.status = f"读取启动失败：{exc}"
            return False
        self.status = f"motor {self.selected_motor}: 正在读取当前目标..."
        return True

    def poll(self, client, now_s: Optional[float] = None) -> Optional[OperationResult]:
        """Poll service readiness and any in-flight operation."""
        self.connected = self._services_ready(client)
        if self.operation is None:
            if not self.connected:
                self.status = "参数服务断连：等待 /chassis/get_parameters 和 set_parameters_atomically"
            return None
        now = time.monotonic() if now_s is None else now_s
        result = self.operation.poll(now)
        if result is None:
            return None
        motor_id = self.operation.motor_id
        self.operation = None
        if result.successful and result.target_deg is not None:
            self.confirmed_targets[motor_id] = result.target_deg
        self.status = result.message
        return result

    def confirmed_target_text(self) -> str:
        """Return the selected motor target as display text."""
        value = self.confirmed_targets[self.selected_motor]
        return "未知" if value is None else f"{value:.1f} deg"

    @staticmethod
    def _services_ready(client) -> bool:
        try:
            return bool(client.services_ready())
        except Exception:  # noqa: BLE001 - connection checks should not crash the UI.
            return False


class RosParameterClient:
    """Thin wrapper around rcl_interfaces parameter services."""

    def __init__(self, node, remote_node_name: str):
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import GetParameters, SetParametersAtomically

        self.Parameter = Parameter
        self.ParameterType = ParameterType
        self.ParameterValue = ParameterValue
        self.GetParameters = GetParameters
        self.SetParametersAtomically = SetParametersAtomically
        remote = "/" + remote_node_name.strip("/")
        self.get_client = node.create_client(GetParameters, f"{remote}/get_parameters")
        self.set_client = node.create_client(
            SetParametersAtomically, f"{remote}/set_parameters_atomically"
        )

    def services_ready(self) -> bool:
        """Return true when both parameter services are reachable."""
        return (
            self.get_client.wait_for_service(timeout_sec=0.0)
            and self.set_client.wait_for_service(timeout_sec=0.0)
        )

    def get_parameter(self, name: str):
        """Start an async get-parameter request."""
        request = self.GetParameters.Request()
        request.names = [name]
        return self.get_client.call_async(request)

    def get_parameter_result(self, future, name: str) -> float:
        """Extract a double parameter from a completed get future."""
        response = future.result()
        if response is None or len(response.values) != 1:
            raise RuntimeError(f"{name} 未返回值")
        value = response.values[0]
        if value.type != self.ParameterType.PARAMETER_DOUBLE:
            raise RuntimeError(f"{name} 不是 double 参数")
        return value.double_value

    def set_parameter(self, name: str, value: float):
        """Start an async atomic set for one double parameter."""
        request = self.SetParametersAtomically.Request()
        parameter = self.Parameter()
        parameter.name = name
        parameter.value = self.ParameterValue(
            type=self.ParameterType.PARAMETER_DOUBLE,
            double_value=float(value),
        )
        request.parameters = [parameter]
        return self.set_client.call_async(request)

    def set_parameter_result(self, future):
        """Extract success and reason from a completed atomic set future."""
        response = future.result()
        if response is None:
            raise RuntimeError("set_parameters_atomically 未返回响应")
        result = response.result
        return bool(result.successful), result.reason

    def remove_pending_request(self, future) -> None:
        """Drop a pending future from the underlying ROS clients when possible."""
        for client in (self.get_client, self.set_client):
            try:
                client.remove_pending_request(future)
            except (AttributeError, KeyError, ValueError):
                pass


class RosRuntime:
    """Own the rclpy node lifecycle for the terminal UI."""

    def __init__(self, remote_node_name: str):
        import rclpy

        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("can3_control_terminal")
        self.client = RosParameterClient(self.node, remote_node_name)

    def spin_once(self, timeout_s: float = 0.0) -> None:
        """Spin the UI node once."""
        self.rclpy.spin_once(self.node, timeout_sec=timeout_s)

    def close(self) -> None:
        """Destroy the local UI node without affecting the remote chassis node."""
        self.node.destroy_node()
        self.rclpy.shutdown()


def safe_call(action, *args) -> bool:
    """Run a curses call and ignore display-size errors."""
    try:
        action(*args)
        return True
    except curses.error:
        return False


def render(screen, model: ControlModel, node_name: str) -> None:
    """Render the terminal UI without letting curses display errors exit the program."""
    safe_call(screen.erase)
    connected = "已连接" if model.connected else "未连接"
    inflight = "有在途操作" if model.operation is not None else "空闲"
    lines = [
        "CAN3 电机角度目标微调",
        f"连接: {connected}  远端节点: {node_name}  状态: {inflight}",
        f"所选电机: {model.selected_motor}  最近确认目标: {model.confirmed_target_text()}",
        f"固定步长: {STEP_DEG:.0f} deg  Kp={KP:.0f} Kd={KD:.0f}  "
        f"整车断电阈值: {SPEED_LIMIT_RPM:.0f} rpm",
        f"编码位置范围约 ±{ENCODING_RANGE_DEG:.3f} deg；"
        "这不是机械安全限位，请操作者留意机械行程。",
        "按键: 1/2/3 选择电机，↑ 增加 1 deg，↓ 减少 1 deg，q 退出。",
        "界面不会自动调角；退出只关闭本终端，不 shutdown 远程节点。反馈详见主终端日志。",
        "",
        model.status,
    ]
    height, width = screen.getmaxyx()
    if height <= 0 or width <= 1:
        return
    for row, line in enumerate(lines[:height]):
        safe_call(screen.addnstr, row, 0, line, max(0, width - 1))
    safe_call(screen.refresh)


def drain_keys(screen) -> list[int]:
    """Drain already buffered key events from curses."""
    keys = []
    while True:
        try:
            key = screen.getch()
        except curses.error:
            break
        if key == -1:
            break
        keys.append(key)
    return keys


def handle_keys(keys: list[int], model: ControlModel, client) -> bool:
    """Apply a drained key batch; return true when the UI should exit."""
    if any(key in (ord("q"), ord("Q")) for key in keys):
        return True

    if model.operation is not None:
        if any(key in (curses.KEY_UP, curses.KEY_DOWN) for key in keys):
            model.status = "已有一次参数操作在途，忽略方向键以防堆积"
        return False

    for key in keys:
        if key in (ord("1"), ord("2"), ord("3")):
            model.select_motor(key - ord("0"))
        elif key == curses.KEY_UP:
            model.begin_adjust(client, +1)
            return False
        elif key == curses.KEY_DOWN:
            model.begin_adjust(client, -1)
            return False
    return False


def run_curses(screen, runtime: RosRuntime, args) -> None:
    """Run the curses event loop."""
    try:
        curses.curs_set(0)
    except curses.error:
        pass
    screen.nodelay(True)
    screen.timeout(0)
    model = ControlModel(timeout_s=args.timeout)
    try:
        while True:
            if handle_keys(drain_keys(screen), model, runtime.client):
                return
            runtime.spin_once(0.05)
            model.poll(runtime.client)
            render(screen, model, args.node)
    finally:
        render(screen, model, args.node)


def positive_finite(value: str) -> float:
    """Parse a finite positive timeout value."""
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("--timeout must be a finite positive number") from exc
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("--timeout must be a finite positive number")
    return parsed


def parse_args(argv=None):
    """Parse command-line arguments without importing ROS modules."""
    parser = argparse.ArgumentParser(
        description="CAN3 motor 1/2/3 target-angle terminal controller."
    )
    parser.add_argument(
        "--node", default="/chassis", help="remote chassis node name (default: /chassis)"
    )
    parser.add_argument(
        "--timeout",
        type=positive_finite,
        default=2.0,
        help="seconds before an in-flight parameter operation is marked unknown",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    """Run the terminal controller."""
    args = parse_args(argv)
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        print("CAN3 控制界面需要在交互式 TTY 终端中运行；未发送任何参数。", file=sys.stderr)
        return 2
    runtime = None
    try:
        runtime = RosRuntime(args.node)
        curses.wrapper(lambda screen: run_curses(screen, runtime, args))
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        if runtime is not None:
            runtime.close()


if __name__ == "__main__":
    raise SystemExit(main())
