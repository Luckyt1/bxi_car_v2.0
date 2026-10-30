#!/usr/bin/env python3
"""读取 Linux 手柄按钮编号及按下/松开状态；仅使用 Python 标准库。"""

import argparse
import errno
import fcntl
import os
from pathlib import Path
import select
import struct
import sys
import time


# Linux <linux/joystick.h>: uint32 time, int16 value, uint8 type, uint8 number.
EVENT = struct.Struct("=IhBB")
BUTTON, AXIS, INIT = 0x01, 0x02, 0x80


def device_info(fd):
    name = bytearray(256)
    axes, buttons = bytearray(1), bytearray(1)
    try:
        fcntl.ioctl(fd, 0x81006A13, name)  # JSIOCGNAME(256)
        fcntl.ioctl(fd, 0x80016A11, axes)  # JSIOCGAXES
        fcntl.ioctl(fd, 0x80016A12, buttons)  # JSIOCGBUTTONS
    except OSError as error:
        if error.errno != errno.ENOTTY:
            raise
        return "未知设备（不能查询名称）"
    label = bytes(name).split(b"\0", 1)[0].decode("utf-8", errors="replace")
    return f"{label}，{buttons[0]} 个按钮，{axes[0]} 个轴"


def available_devices():
    return sorted((p for p in Path("/dev/input").glob("js*")
                   if p.name[2:].isdigit()), key=lambda p: int(p.name[2:]))


def list_devices(devices):
    if not devices:
        print("未检测到 /dev/input/js* 手柄设备，请先连接手柄。")
    for device in devices:
        try:
            fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
            try:
                description = device_info(fd)
            finally:
                os.close(fd)
        except OSError as error:
            description = f"无法读取：{error.strerror}"
        print(f"{device}: {description}")


def monitor(device, show_axes=False):
    fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
    try:
        print(f"设备：{device}\n名称：{device_info(fd)}", flush=True)
        print("请逐个按下、松开按钮；编号从 0 开始，与机器人 button 编号一致。", flush=True)
        print("Ctrl-C 退出。方向键若没有按钮输出，可加 --axes 查看轴事件。", flush=True)
        pending = bytearray()
        axis_values = {}
        while True:
            if not select.select([fd], [], [], 0.5)[0]:
                continue
            try:
                data = os.read(fd, EVENT.size * 64)
            except BlockingIOError:
                continue
            if not data:
                print("设备已断开。", flush=True)
                return 1
            pending.extend(data)
            while len(pending) >= EVENT.size:
                _, value, event_type, number = EVENT.unpack_from(pending)
                del pending[:EVENT.size]
                initial = bool(event_type & INIT)
                kind = event_type & ~INIT
                prefix = "初始化" if initial else time.strftime("%H:%M:%S")
                if kind == BUTTON:
                    state = "按下" if value else "松开"
                    print(f"[{prefix}] button{number:<3} {state}  value={value}", flush=True)
                elif kind == AXIS and show_axes:
                    normalized = max(-1.0, min(1.0, value / 32767.0))
                    previous = axis_values.get(number)
                    if (initial or previous is None or abs(normalized - previous) >= 0.05
                            or (value == 0 and previous != 0)):
                        axis_values[number] = normalized
                        print(f"[{prefix}] axis{number:<3} {normalized:+.3f}  raw={value}", flush=True)
    finally:
        os.close(fd)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", "-d", type=Path, help="设备路径，例如 /dev/input/js0")
    parser.add_argument("--list", action="store_true", help="列出本地手柄后退出")
    parser.add_argument("--axes", action="store_true", help="同时显示摇杆、方向键等轴事件")
    args = parser.parse_args(argv)
    devices = available_devices()
    if args.list:
        list_devices(devices)
        return 0
    device = args.device
    if device is None:
        if len(devices) != 1:
            list_devices(devices)
            if devices:
                print("发现多个设备，请使用 --device 指定要测试的手柄。")
            return 1
        device = devices[0]
    try:
        return monitor(device, args.axes)
    except KeyboardInterrupt:
        print("\n按键测试已退出。")
        return 0
    except PermissionError:
        print(f"没有读取 {device} 的权限；可用 sudo python3 运行本脚本。", file=sys.stderr)
    except FileNotFoundError:
        print(f"设备不存在：{device}；请先连接手柄或用 --list 查看。", file=sys.stderr)
    except OSError as error:
        print(f"读取设备失败：{error.strerror}（可能已断开）", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
