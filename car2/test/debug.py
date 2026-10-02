#!/usr/bin/env python3
"""数字调试入口；--dry-run 仅显示命令，机械臂解算不访问硬件。"""

import argparse
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys


def source_root():
    """Locate this source layout, without accepting historical build directories."""
    here = Path(__file__).resolve().parent
    if here.name == "chassis" and here.parent.name == "lib":
        return None
    for parent in here.parents:
        if (parent / "test" / "debug.py").is_file() and (parent / "src" / "thirdparty").is_dir():
            return parent
    return None


def resolve_tool(name, script=False, dry_run=False):
    """Use installed siblings or the project's standard colcon outputs."""
    here = Path(__file__).resolve().parent
    root = source_root()
    if root is None:
        candidates = [here / name]
    elif script:
        candidates = [root / "test" / "tools" / name]
    else:
        build = Path(os.environ.get("CHASSIS_BUILD_DIR", root / "build"))
        install = Path(os.environ.get("CHASSIS_INSTALL_DIR", root / "install"))
        candidates = [build / "chassis" / name, build / name,
                      install / "chassis" / "lib" / "chassis" / name,
                      install / "lib" / "chassis" / name]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    if dry_run:
        return candidates[0]
    raise ValueError(f"找不到 {name}，请先构建；查找位置：" + ", ".join(map(str, candidates)))


def default_config(name):
    """Resolve the package's source or installed configuration."""
    root = source_root()
    if root is not None:
        return root / "src" / "config" / name
    return Path(__file__).resolve().parents[2] / "share" / "chassis" / "config" / name


def integer(value):
    """Accept decimal or 0x-prefixed CAN identifiers."""
    try:
        return int(value, 16 if value.lower().startswith("0x") else 10)
    except ValueError as error:
        raise argparse.ArgumentTypeError("请输入整数或 0x 十六进制数") from error


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument(
        "--mode", choices=("bxi", "yiyou", "iswv", "arm", "chassis", "chassis-remote"),
        help="直接选择功能；省略时显示数字菜单")
    parser.add_argument("--interface", help="意优 EtherCAT 物理网卡")
    parser.add_argument("--slaves", default="1,2,3", help="意优物理站位，逗号分隔，范围 1..199")
    parser.add_argument("--bus", type=integer, help="CAN 总线/共享电源句柄 0..6；BXI/意优默认 3，ISWV 默认 1")
    parser.add_argument("--node", type=integer, default=1, help="BXI CAN ID 或 ISWV Node-ID，默认 1")
    parser.add_argument("--model", choices=("5014", "5018", "7010", "8515"), default="5014",
                        help="BXI MIT 编码型号，默认 5014")
    parser.add_argument("--config", type=Path, help="底盘 steering.yaml 或机械臂 arm_kinematics.json")
    parser.add_argument("--yiyou-mode", choices=("jog", "home"), default="jog")
    parser.add_argument("--dry-run", action="store_true", help="仅打印命令，不运行工具或初始化硬件")
    result = parser.parse_args(argv)
    if result.bus is not None and not 0 <= result.bus <= 6:
        parser.error("--bus 必须在 0..6")
    if not 0 <= result.node <= 0x7ff:
        parser.error("--node 必须在 0..2047")
    return result


def choose(prompt, choices):
    while True:
        try:
            value = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            return None
        if value == "0":
            return None
        if value in choices:
            return choices[value]
        print("输入无效，请输入菜单数字；0 退出。")


def select_mode():
    mode = choose("\n1 电机调试\n2 机械臂解算（离线 FK/IK）\n3 底盘调试\n0 退出\n选择：",
                  {"1": "motor", "2": "arm", "3": "chassis"})
    if mode == "motor":
        return choose("\n1 BXI\n2 意优 Yiyou（EtherCAT）\n3 ISWV\n0 退出\n选择品牌：",
                      {"1": "bxi", "2": "yiyou", "3": "iswv"})
    if mode == "chassis":
        return choose("\n1 直接启动底盘\n2 等待遥控启动底盘\n0 退出\n选择：",
                      {"1": "chassis", "2": "chassis-remote"})
    return mode


def command_for(mode, args):
    """Validate a route and return argv; this function never starts a process."""
    if mode in ("bxi", "iswv"):
        if mode == "iswv" and not 1 <= args.node <= 127:
            raise ValueError("ISWV --node 必须在 1..127")
        bus = args.bus if args.bus is not None else (1 if mode == "iswv" else 3)
        command = [str(resolve_tool("motor_console", dry_run=args.dry_run)), "--brand", mode,
                   "--bus", str(bus), "--node", str(args.node)]
        if mode == "bxi":
            command.extend(["--model", args.model])
        return command
    if mode == "yiyou":
        if (not args.interface or args.interface == "lo" or
                not re.fullmatch(r"[A-Za-z0-9_.:-]{1,15}", args.interface)):
            raise ValueError("意优需要 --interface 指定物理网卡，例如 --interface enp1s0")
        try:
            slaves = [int(part) for part in args.slaves.split(",")]
        except ValueError as error:
            raise ValueError("--slaves 必须是逗号分隔的整数") from error
        if not slaves or len(set(slaves)) != len(slaves) or any(not 1 <= n <= 199 for n in slaves):
            raise ValueError("--slaves 必须是互不重复的 1..199 物理站位")
        frontend = resolve_tool("yiyou_control.py", script=True, dry_run=args.dry_run)
        return [sys.executable, str(frontend),
                args.yiyou_mode, "--interface", args.interface, "--slaves", args.slaves,
                "--power-bus", str(args.bus if args.bus is not None else 3),
                "--backend", str(resolve_tool("arm_yiyou_control", dry_run=args.dry_run))]
    config_name = "arm_kinematics.json" if mode == "arm" else "steering.yaml"
    config = args.config or default_config(config_name)
    if not config.is_file():
        raise ValueError(f"配置文件不存在：{config}")
    if mode == "arm":
        return [str(resolve_tool("arm_solver", dry_run=args.dry_run)),
                "--config", str(config.resolve())]
    if mode not in ("chassis", "chassis-remote"):
        raise ValueError(f"未知调试功能：{mode}")
    binary = "chassis_only_remote" if mode == "chassis-remote" else "chassis_only"
    return [str(resolve_tool(binary, dry_run=args.dry_run)), "--ros-args", "-p",
            "steering_config_file:=" + str(config.resolve())]


def run_command(command):
    """Forward one shutdown signal and wait for the tool's own cleanup to finish."""
    process = None
    shutdown_signal = None
    delivered = False

    def deliver_pending():
        nonlocal delivered
        if process is not None and shutdown_signal is not None and not delivered:
            delivered = True
            try:
                os.killpg(process.pid, shutdown_signal)
            except ProcessLookupError:
                pass

    def request_shutdown(signum, _frame):
        nonlocal shutdown_signal
        if shutdown_signal is None:
            shutdown_signal = signum
            deliver_pending()

    forwarded = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
    previous = {signum: signal.getsignal(signum) for signum in forwarded}
    try:
        for signum in forwarded:
            signal.signal(signum, request_shutdown)
        # A private session prevents terminal broadcasts plus forwarding from sending
        # SIGINT twice, and limits forwarding to this tool and its own descendants.
        process = subprocess.Popen(command, start_new_session=True)
        deliver_pending()
        while True:
            try:
                return process.wait()
            except KeyboardInterrupt:
                request_shutdown(signal.SIGINT, None)
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def main(argv=None):
    args = arguments(argv)
    mode = args.mode or select_mode()
    if mode is None:
        return 0
    try:
        command = command_for(mode, args)
        print("执行命令：" + shlex.join(command), flush=True)
        if args.dry_run:
            return 0
        return run_command(command)
    except (ValueError, OSError) as error:
        print(f"调试入口：{error}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
