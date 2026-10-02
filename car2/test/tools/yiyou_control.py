#!/usr/bin/env python3
"""Terminal frontend for Yiyou EtherCAT motor jogging, zeroing and homing."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import termios
import time
import tty
from typing import Any


BACKEND_NAME = "arm_yiyou_control"
DEFAULT_SPEED_RPM = 20.0
DEFAULT_TIMEOUT_S = 60.0
CALIBRATION_VERSION = 2
CALIBRATION_PROTOCOL = "yiyou-ethercat-zero"


class UserError(RuntimeError):
    """An expected CLI/runtime error that should be shown without a traceback."""


@dataclass(frozen=True)
class Axis:
    slave: int
    identity: list[int]
    scale: float
    position: int
    current_permille: int | None = None
    rated_current_ma: int = 0

    @property
    def angle_deg(self) -> float:
        return self.position * 360.0 / self.scale


@dataclass
class TerminalState:
    mode: str
    interface: str
    slaves: list[int]
    axes: dict[int, Axis]
    selected_index: int = 0
    busy: str | None = None
    status: str = ""
    last_done: str = ""

    @property
    def selected_slave(self) -> int:
        return self.slaves[self.selected_index]

    def select_delta(self, delta: int) -> None:
        self.selected_index = (self.selected_index + delta) % len(self.slaves)
        self.status = f"selected slave {self.selected_slave}"


class RawTerminal:
    """Own cbreak terminal mode and restore it on every exit path."""

    def __init__(self) -> None:
        self.fd = sys.stdin.fileno()
        self.original = None

    def __enter__(self) -> "RawTerminal":
        self.original = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self.original is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original)


class SignalTrap:
    """Turn process stop signals into cleanup paths that notify the child first."""

    def __init__(self) -> None:
        self.signaled: int | None = None
        self.previous: dict[int, Any] = {}

    def __enter__(self) -> "SignalTrap":
        for signum in (signal.SIGTERM, signal.SIGHUP):
            self.previous[signum] = signal.getsignal(signum)
            signal.signal(signum, self._handle)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        for signum, handler in self.previous.items():
            signal.signal(signum, handler)

    def _handle(self, signum, frame) -> None:
        del frame
        self.signaled = signum
        raise KeyboardInterrupt


class KeyReader:
    """Read terminal keys while preserving split escape sequences between polls."""

    def __init__(self) -> None:
        self.buffer = b""

    def read(self, timeout_s: float = 0.1) -> list[str]:
        ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if ready:
            data = os.read(sys.stdin.fileno(), 64)
            if not data:
                raise EOFError
            self.buffer += data
        return self._parse_complete()

    def _parse_complete(self) -> list[str]:
        keys: list[str] = []
        i = 0
        while i < len(self.buffer):
            byte = self.buffer[i]
            if byte == 3:
                keys.append("ctrl_c")
                i += 1
            elif byte in (ord("q"), ord("Q")):
                keys.append("quit")
                i += 1
            elif byte in (ord("z"), ord("Z")):
                keys.append("zero")
                i += 1
            elif byte == 27:
                if len(self.buffer) - i < 3:
                    if len(self.buffer) - i == 1 or self.buffer[i + 1] == ord("["):
                        break
                    i += 1
                    continue
                if self.buffer[i + 1] != ord("["):
                    i += 1
                    continue
                sequence = self.buffer[i:i + 3]
                if sequence in (b"\x1b[A", b"\x1b[B", b"\x1b[C", b"\x1b[D"):
                    code = sequence[2]
                    keys.append({65: "up", 66: "down", 67: "right", 68: "left"}[code])
                    i += 3
                else:
                    i += 1
            else:
                i += 1
        self.buffer = self.buffer[i:]
        return keys

    def discard_pending(self) -> None:
        self.buffer = b""


class BackendProcess:
    """Line-oriented JSON wrapper for the persistent C++ backend."""

    def __init__(self, args: argparse.Namespace) -> None:
        command = [
            str(args.backend),
            "--interface",
            args.interface,
            "--slaves",
            ",".join(str(slave) for slave in args.slaves),
            "--power-bus",
            str(args.power_bus),
            "--speed-rpm",
            format_float(args.speed_rpm),
            "--timeout",
            format_float(args.timeout),
        ]
        if args.mode == "jog":
            command.append("--current-feedback")
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=None,
            bufsize=0,
        )
        assert self.process.stdout is not None
        os.set_blocking(self.process.stdout.fileno(), False)
        self._stdout_buffer = b""

    def send(self, line: str) -> None:
        if self.process.stdin is None or self.process.poll() is not None:
            raise UserError("backend is not running")
        try:
            self.process.stdin.write((line + "\n").encode("utf-8"))
            self.process.stdin.flush()
        except BrokenPipeError as exc:
            raise UserError("backend closed its input") from exc

    def signal_interrupt(self) -> None:
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)

    def terminate(self) -> None:
        if self.process.poll() is None:
            self.process.terminate()

    def close(self) -> None:
        for pipe in (self.process.stdin, self.process.stdout):
            if pipe is None:
                continue
            try:
                pipe.close()
            except OSError:
                pass

    def wait(self, timeout_s: float | None = None) -> int | None:
        try:
            return self.process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            return None

    def poll_events(self) -> list[dict[str, Any]]:
        events: list[dict[str, Any]] = []
        if self.process.stdout is None:
            return events
        fd = self.process.stdout.fileno()
        while True:
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                break
            if not chunk:
                break
            self._stdout_buffer += chunk
            while b"\n" in self._stdout_buffer:
                raw_line, self._stdout_buffer = self._stdout_buffer.split(b"\n", 1)
                line = raw_line.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                try:
                    event = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise UserError(f"backend stdout is not JSON: {line}") from exc
                if not isinstance(event, dict):
                    raise UserError(f"backend stdout JSON is not an object: {line}")
                events.append(event)
        return events


def format_float(value: float) -> str:
    return f"{value:g}"


def parse_slaves(value: str) -> list[int]:
    if not value.strip():
        raise argparse.ArgumentTypeError("--slaves must list one or more physical positions")
    slaves: list[int] = []
    for part in value.split(","):
        part = part.strip()
        if not part:
            raise argparse.ArgumentTypeError("--slaves must not contain empty entries")
        try:
            slave = int(part, 10)
        except ValueError as exc:
            raise argparse.ArgumentTypeError("slave positions must be integers") from exc
        if slave < 1 or slave > 199:
            raise argparse.ArgumentTypeError("slave positions must be in range 1..199")
        if slave in slaves:
            raise argparse.ArgumentTypeError("slave positions must be unique")
        slaves.append(slave)
    return slaves


def parse_bus(value: str) -> int:
    try:
        bus = int(value, 10)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("--power-bus must be an integer in range 0..6") from exc
    if bus < 0 or bus > 6:
        raise argparse.ArgumentTypeError("--power-bus must be in range 0..6")
    return bus


def parse_speed(value: str) -> float:
    try:
        speed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("--speed-rpm must be a finite number") from exc
    if not math.isfinite(speed) or speed <= 0.0 or speed >= 60.0:
        raise argparse.ArgumentTypeError("--speed-rpm must be > 0 and < 60")
    return speed


def parse_timeout(value: str) -> float:
    try:
        timeout = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("--timeout must be a finite number") from exc
    if not math.isfinite(timeout) or timeout <= 0.0 or timeout > 3600.0:
        raise argparse.ArgumentTypeError("--timeout must be > 0 and <= 3600")
    return timeout


def repo_root() -> Path | None:
    here = Path(__file__).resolve().parent
    if here.name == "chassis" and here.parent.name == "lib":
        return None
    for parent in here.parents:
        if (parent / 'test' / 'tools' / 'yiyou_control.py').is_file():
            return parent
    return None


def default_calibration_file() -> Path:
    return (repo_root() or Path.cwd()) / "yiyou_ethercat_zero_calibration.json"


def find_backend() -> Path:
    root = repo_root()
    candidates = [
        Path(__file__).resolve().with_name(BACKEND_NAME),
    ]
    if root is not None:
        build = Path(os.environ.get("CHASSIS_BUILD_DIR", root / "build"))
        install = Path(os.environ.get("CHASSIS_INSTALL_DIR", root / "install"))
        candidates += [
            build / "chassis" / BACKEND_NAME,
            build / BACKEND_NAME,
            install / "chassis" / "lib" / "chassis" / BACKEND_NAME,
            install / "lib" / "chassis" / BACKEND_NAME,
        ]
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate
    searched = "\n".join(f"  {path}" for path in candidates)
    raise UserError(
        f"could not find {BACKEND_NAME}; build chassis first, then retry.\nSearched:\n{searched}"
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    raw_args = sys.argv[1:] if argv is None else argv
    retired = {"--bus", "--node", "--nodes"} & {
        argument.split('=', 1)[0] for argument in raw_args
    }
    if retired:
        option = sorted(retired)[0]
        raise UserError(
            f"{option} is a retired CAN option; use --interface NIC and --slaves positions"
        )
    parser = argparse.ArgumentParser(
        description="Interactive terminal frontend for Yiyou EtherCAT arm motors.",
        allow_abbrev=False,
    )
    subparsers = parser.add_subparsers(dest="mode", required=True)
    for name in ("jog", "home"):
        sub = subparsers.add_parser(
            name, allow_abbrev=False,
            description=f"Yiyou EtherCAT {name}; select a NIC and physical slave positions.",
        )
        sub.add_argument("--interface", required=True, help="EtherCAT NIC, for example eth0")
        sub.add_argument(
            "--slaves",
            required=True,
            type=parse_slaves,
            help="comma-separated unique physical EtherCAT positions, each 1..199",
        )
        sub.add_argument(
            "--power-bus",
            default=3,
            type=parse_bus,
            help="BXI shared-power handle only; motor commands use EtherCAT (default: 3)",
        )
        sub.add_argument(
            "--speed-rpm",
            default=DEFAULT_SPEED_RPM,
            type=parse_speed,
            help="output-shaft motion speed, >0 and <60 rpm (default: 1)",
        )
        sub.add_argument(
            "--timeout",
            default=DEFAULT_TIMEOUT_S,
            type=parse_timeout,
            help="backend command timeout seconds, >0 and <=3600 (default: 60)",
        )
        sub.add_argument(
            "--calibration-file",
            type=Path,
            default=default_calibration_file(),
            help="persistent zero record JSON path",
        )
        sub.add_argument(
            "--backend",
            type=Path,
            default=None,
            help=f"path to {BACKEND_NAME}; auto-detected when omitted",
        )
    args = parser.parse_args(argv)
    if (not args.interface or args.interface == "lo" or len(args.interface) > 15
            or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.:-"
                   for c in args.interface)):
        parser.error("specify a physical EtherCAT --interface; lo is loopback")
    return args


def terminal_required() -> None:
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        raise UserError("interactive TTY required; no motor command was sent")


def record_key(interface: str, slave: int) -> str:
    return f"{interface}:{slave}"


def load_calibration(path: Path, *, allow_missing: bool) -> dict[str, Any]:
    if not path.exists():
        if allow_missing:
            return {
                "version": CALIBRATION_VERSION,
                "protocol": CALIBRATION_PROTOCOL,
                "records": {},
            }
        raise UserError(f"calibration file not found: {path}")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise UserError(f"calibration file is malformed; leaving it untouched: {path}") from exc
    validate_calibration_document(data)
    return data


def validate_calibration_document(data: Any) -> None:
    if (
        not isinstance(data, dict)
        or data.get("version") != CALIBRATION_VERSION
        or data.get("protocol") != CALIBRATION_PROTOCOL
    ):
        raise UserError(
            "calibration file must be EtherCAT version 2; old CAN zero records are refused"
        )
    records = data.get("records")
    if not isinstance(records, dict):
        raise UserError("calibration file records must be an object")
    for key, record in records.items():
        if not isinstance(key, str) or not isinstance(record, dict):
            raise UserError("calibration file contains a malformed record")
        identity = record.get("identity")
        scale = record.get("scale")
        if (
            not isinstance(identity, list)
            or len(identity) != 4
            or not all(isinstance(item, int) for item in identity)
            or not isinstance(scale, (int, float))
            or isinstance(scale, bool)
            or not math.isfinite(float(scale))
            or float(scale) <= 0.0
        ):
            raise UserError(f"calibration record {key} is malformed; leaving file untouched")


def atomic_write_json(path: Path, data: dict[str, Any]) -> None:
    validate_calibration_document(data)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    payload = json.dumps(data, indent=2, sort_keys=True) + "\n"
    try:
        with temp.open("w", encoding="utf-8") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp, path)
        directory_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        try:
            if temp.exists():
                temp.unlink()
        except OSError:
            pass


def calibration_document(records: dict[str, Any]) -> dict[str, Any]:
    return {
        "version": CALIBRATION_VERSION,
        "protocol": CALIBRATION_PROTOCOL,
        "records": records,
    }


def remove_record_before_zero(path: Path, interface: str, slave: int) -> dict[str, Any]:
    data = load_calibration(path, allow_missing=True)
    records = dict(data["records"])
    records.pop(record_key(interface, slave), None)
    updated = calibration_document(records)
    atomic_write_json(path, updated)
    return updated


def persist_record(path: Path, interface: str, axis: Axis) -> None:
    data = load_calibration(path, allow_missing=True)
    records = dict(data["records"])
    records[record_key(interface, axis.slave)] = {
        "identity": list(axis.identity),
        "scale": axis.scale,
    }
    atomic_write_json(path, calibration_document(records))


def require_home_records(args: argparse.Namespace) -> dict[str, Any]:
    data = load_calibration(args.calibration_file, allow_missing=False)
    missing = [
        record_key(args.interface, slave)
        for slave in args.slaves
        if record_key(args.interface, slave) not in data["records"]
    ]
    if missing:
        raise UserError(
            "home refused because zero records are missing: " + ", ".join(missing)
        )
    return data


def axis_from_json(item: Any) -> Axis:
    if not isinstance(item, dict):
        raise UserError("backend ready axes must contain objects")
    try:
        slave = int(item["slave"])
        identity = item["identity"]
        scale = float(item["scale"])
        position = int(item["position"])
    except (KeyError, TypeError, ValueError) as exc:
        raise UserError("backend ready axis is missing slave/identity/scale/position") from exc
    if (
        not isinstance(identity, list)
        or len(identity) != 4
        or not all(isinstance(part, int) for part in identity)
    ):
        raise UserError(f"backend ready slave {slave} has invalid identity")
    if not math.isfinite(scale) or scale <= 0.0:
        raise UserError(f"backend ready slave {slave} has invalid scale")
    return Axis(slave=slave, identity=list(identity), scale=scale, position=position)


def wait_ready(
    backend: BackendProcess,
    expected_slaves: list[int],
    timeout_s: float = 30.0,
) -> dict[int, Axis]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for event in backend.poll_events():
            if event.get("event") == "error":
                raise UserError(str(event.get("message", "backend error")))
            if event.get("event") != "ready":
                continue
            raw_axes = event.get("axes")
            if not isinstance(raw_axes, list):
                raise UserError("backend ready event has no axes list")
            axes = {axis.slave: axis for axis in (axis_from_json(item) for item in raw_axes)}
            if sorted(axes) != sorted(expected_slaves):
                raise UserError(
                    f"backend ready slaves {sorted(axes)} do not match requested {expected_slaves}"
                )
            return axes
        exit_code = backend.process.poll()
        if exit_code is not None:
            raise UserError(f"backend exited before ready with code {exit_code}")
        time.sleep(0.02)
    raise UserError("timed out waiting for backend ready")


def verify_home_records(
    args: argparse.Namespace,
    axes: dict[int, Axis],
    calibration: dict[str, Any],
) -> None:
    for slave in args.slaves:
        key = record_key(args.interface, slave)
        record = calibration["records"][key]
        axis = axes[slave]
        if record["identity"] != axis.identity:
            raise UserError(f"home refused: identity mismatch for {key}")
        if float(record["scale"]) != axis.scale:
            raise UserError(f"home refused: position scale mismatch for {key}")


def render(state: TerminalState) -> None:
    lines = []
    selected = state.selected_slave
    title = "意优 EtherCAT 电机点动/调零" if state.mode == "jog" else "意优 EtherCAT 电机回零"
    lines.append(f"{title}  网口={state.interface}")
    lines.append(f"物理站位={','.join(str(n) for n in state.slaves)}  当前站位={selected}")
    for slave in state.slaves:
        axis = state.axes[slave]
        marker = ">" if slave == selected else " "
        lines.append(
            f"{marker} Slave {slave:3d}  输出角={axis.angle_deg:9.3f} deg  "
            f"位置={axis.position:10d} pulse"
        )
        if state.mode == "jog":
            if axis.current_permille is None:
                lines.append("    实际电流: 等待读取")
            else:
                current = axis.current_permille
                amperes = (f"{current * axis.rated_current_ma / 1_000_000:.3f} A"
                           if axis.rated_current_ma > 0 else "A 未知（额定电流不可用）")
                lines.append(
                    f"    实际电流: {current:+d} ‰ ({current / 10:+.1f}% 额定)  "
                    f"{amperes}"
                )
    if state.mode == "jog":
        lines.append("按键: ↑/↓ 当前 slave 每次 +/-1 输出轴度；←/→ 切换 slave；q 退出")
        lines.append("      z 把当前 slave 的当前位置写成 EEPROM 零点")
        lines.append(
            f"注意: z 会覆盖 slave {selected} 的 EEPROM 零点并保存全部持久参数。"
        )
    else:
        if state.busy:
            lines.append("正在使能并回到已保存零点，完成后保持使能。")
        else:
            lines.append("已回到零点并保持使能。")
        lines.append("按键: q 退出")
    busy = state.busy or "空闲"
    lines.append(f"状态: {busy}  {state.status}")
    if state.last_done:
        lines.append(state.last_done)
    sys.stdout.write("\x1b[2J\x1b[H" + "\n".join(lines) + "\n")
    sys.stdout.flush()


def apply_done_event(
    args: argparse.Namespace,
    state: TerminalState,
    event: dict[str, Any],
) -> None:
    command = str(event.get("command", ""))
    slave_value = event.get("slave")
    if isinstance(slave_value, int) and slave_value in state.axes:
        old = state.axes[slave_value]
        position = int(event.get("position", old.position))
        state.axes[slave_value] = replace(old, position=position)
    if command == "zero":
        slave = int(event["slave"])
        persist_record(args.calibration_file, args.interface, state.axes[slave])
        state.last_done = f"已保存零点: {record_key(args.interface, slave)}"
    elif command == "home":
        for slave, old in list(state.axes.items()):
            state.axes[slave] = replace(old, position=0)
        state.last_done = "回零完成，正在保持零点"
    else:
        state.last_done = f"{command} 完成"
    state.busy = None
    state.status = "就绪"


def handle_backend_events(
    args: argparse.Namespace,
    state: TerminalState,
    backend: BackendProcess,
) -> None:
    saw_event = False
    for event in backend.poll_events():
        saw_event = True
        if event.get("event") == "error":
            raise UserError(str(event.get("message", "backend error")))
        if event.get("event") == "done":
            apply_done_event(args, state, event)
        elif event.get("event") == "feedback":
            slave = int(event["slave"])
            if slave not in state.axes:
                raise UserError(f"backend feedback has unexpected slave {slave}")
            state.axes[slave] = replace(
                state.axes[slave], position=int(event["position"]),
                current_permille=int(event["current_permille"]),
                rated_current_ma=int(event["rated_current_ma"]),
            )
    exit_code = backend.process.poll()
    if exit_code is not None and not saw_event:
        raise UserError(f"backend exited with code {exit_code}")


def cleanup_wait_s(args: argparse.Namespace) -> float:
    return max(30.0, 10.0 + 5.0 * len(args.slaves))


def wait_backend_exit(args: argparse.Namespace, backend: BackendProcess) -> int:
    code = backend.wait(cleanup_wait_s(args))
    if code is not None:
        return int(code)
    backend.terminate()
    code = backend.wait(cleanup_wait_s(args))
    if code is not None:
        raise UserError(f"backend did not exit after graceful quit; SIGTERM returned {code}")
    raise UserError("backend is still running after graceful shutdown and SIGTERM")


def run_jog(args: argparse.Namespace, backend: BackendProcess, axes: dict[int, Axis]) -> None:
    state = TerminalState(
        mode="jog", interface=args.interface, slaves=args.slaves, axes=axes, status="保持当前位置")
    reader = KeyReader()
    backend.send("hold")
    state.busy = "hold"
    with RawTerminal():
        while True:
            if not sys.stdin.isatty():
                raise EOFError
            was_busy = state.busy is not None
            keys = reader.read()
            if was_busy:
                if any(key in ("quit", "ctrl_c") for key in keys):
                    backend.signal_interrupt()
                    raise KeyboardInterrupt
                if any(key in ("up", "down") for key in keys):
                    state.status = "忙碌中，已丢弃排队的方向键"
                reader.discard_pending()
                handle_backend_events(args, state, backend)
                render(state)
                continue
            handle_backend_events(args, state, backend)
            if any(key in ("quit", "ctrl_c") for key in keys):
                if "ctrl_c" in keys:
                    backend.signal_interrupt()
                    raise KeyboardInterrupt
                backend.send("quit")
                return
            for key in keys:
                if key == "left":
                    state.select_delta(-1)
                elif key == "right":
                    state.select_delta(+1)
                elif key in ("up", "down"):
                    sign = 1 if key == "up" else -1
                    slave = state.selected_slave
                    backend.send(f"step {slave} {sign}")
                    state.busy = f"step slave {slave}"
                    state.status = f"已发送 {sign:+d} 度"
                    break
                elif key == "zero":
                    slave = state.selected_slave
                    remove_record_before_zero(args.calibration_file, args.interface, slave)
                    backend.send(f"zero {slave}")
                    state.busy = f"zero slave {slave}"
                    state.status = "旧记录已失效，正在写 EEPROM 零点"
                    break
            render(state)


def run_home(args: argparse.Namespace, backend: BackendProcess, axes: dict[int, Axis]) -> None:
    state = TerminalState(
        mode="home", interface=args.interface, slaves=args.slaves, axes=axes, status="正在回零")
    reader = KeyReader()
    backend.send("home")
    state.busy = "home"
    with RawTerminal():
        while True:
            if not sys.stdin.isatty():
                raise EOFError
            was_busy = state.busy is not None
            keys = reader.read()
            if any(key in ("quit", "ctrl_c") for key in keys):
                if was_busy or "ctrl_c" in keys:
                    backend.signal_interrupt()
                    raise KeyboardInterrupt
                backend.send("quit")
                return
            handle_backend_events(args, state, backend)
            render(state)


def run(args: argparse.Namespace) -> int:
    terminal_required()
    if args.backend is None:
        args.backend = find_backend()
    if not args.backend.is_file() or not os.access(args.backend, os.X_OK):
        raise UserError(f"backend is not executable: {args.backend}")
    if args.mode == "jog":
        load_calibration(args.calibration_file, allow_missing=True)
    calibration = require_home_records(args) if args.mode == "home" else None
    backend = BackendProcess(args)
    try:
        with SignalTrap():
            axes = wait_ready(backend, args.slaves)
            if args.mode == "home":
                assert calibration is not None
                verify_home_records(args, axes, calibration)
                run_home(args, backend, axes)
            else:
                run_jog(args, backend, axes)
            code = wait_backend_exit(args, backend)
            return 0 if code == 0 else code
    except EOFError as exc:
        backend.signal_interrupt()
        wait_backend_exit(args, backend)
        raise UserError("terminal input closed; backend was interrupted") from exc
    except KeyboardInterrupt:
        backend.signal_interrupt()
        wait_backend_exit(args, backend)
        return 130
    except Exception:
        backend.signal_interrupt()
        wait_backend_exit(args, backend)
        raise
    finally:
        backend.close()


def main(argv: list[str] | None = None) -> int:
    try:
        args = parse_args(argv)
        return run(args)
    except (UserError, OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
