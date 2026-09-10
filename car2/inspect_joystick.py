#!/usr/bin/env python3
"""只读显示 Linux joystick 的轴、方向键和按钮事件。"""

import argparse
import array
import fcntl
import glob
import os
import struct
import sys


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JSIOCGAXES = 0x80016A11
JSIOCGBUTTONS = 0x80016A12
EVENT = struct.Struct("<IhBB")


def joystick_name(fd: int) -> str:
    size = 128
    buffer = bytearray(size)
    request = 0x80000000 | (size << 16) | (ord("j") << 8) | 0x13
    try:
        fcntl.ioctl(fd, request, buffer, True)
    except OSError:
        return "未知手柄"
    return buffer.split(b"\0", 1)[0].decode(errors="replace") or "未知手柄"


def joystick_count(fd: int, request: int) -> int:
    value = array.array("B", [0])
    fcntl.ioctl(fd, request, value, True)
    return value[0]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="只读取并显示手柄原始事件，不启动 ROS、不上电、也不控制电机。"
    )
    parser.add_argument(
        "device", nargs="?", default="/dev/input/js0", help="手柄设备，默认 /dev/input/js0"
    )
    parser.add_argument(
        "--threshold",
        type=int,
        default=1000,
        help="轴变化输出阈值（0~32767），默认 1000，用于过滤摇杆抖动",
    )
    parser.add_argument(
        "--show-init", action="store_true", help="同时显示驱动上报的轴和按钮初始值"
    )
    args = parser.parse_args()
    if not 0 <= args.threshold <= 32767:
        parser.error("--threshold 必须在 0~32767 之间")
    return args


def print_instructions() -> None:
    print("\n请一次只操作一个控件，并把对应输出复制给我：")
    print("  1. 右摇杆前推、松开、后拉：确认前进/后退速度轴")
    print("  2. 方向键左、松开、右：确认左转/右转轴")
    print("  3. 按下准备作为急停的按钮：确认急停按钮编号")
    print("\n符号设置参考：")
    print("  前推值为负数 -> JOYSTICK_INVERT_LINEAR=true；为正数 -> false")
    print("  方向键左值为负数 -> JOYSTICK_INVERT_ANGULAR=true；为正数 -> false")
    print("按 Ctrl+C 退出。\n", flush=True)


def run(device: str, threshold: int, show_init: bool) -> int:
    try:
        fd = os.open(device, os.O_RDONLY)
    except OSError as error:
        devices = ", ".join(sorted(glob.glob("/dev/input/js*"))) or "未发现"
        print(f"无法打开 {device}: {error.strerror}；当前 joystick 设备：{devices}", file=sys.stderr)
        return 1

    try:
        name = joystick_name(fd)
        axes = joystick_count(fd, JSIOCGAXES)
        buttons = joystick_count(fd, JSIOCGBUTTONS)
        print(f"已打开 {device}: {name}，轴={axes}，按钮={buttons}")
        print("本脚本只读取手柄，不会启动底盘或发送电机指令。")
        print_instructions()

        last_axes = [0] * axes
        while True:
            data = os.read(fd, EVENT.size)
            if not data:
                print("手柄已断开。", file=sys.stderr)
                return 1
            if len(data) != EVENT.size:
                print(f"收到不完整的 joystick 事件：{len(data)} 字节", file=sys.stderr)
                return 1

            event_time, value, event_type, number = EVENT.unpack(data)
            is_init = bool(event_type & JS_EVENT_INIT)
            event_type &= ~JS_EVENT_INIT

            if event_type == JS_EVENT_AXIS:
                previous = last_axes[number] if number < len(last_axes) else 0
                if number < len(last_axes):
                    last_axes[number] = value
                changed = abs(value - previous) >= threshold or value == 0 or abs(value) >= 32760
                if (show_init or not is_init) and changed:
                    normalized = max(-1.0, min(1.0, value / 32767.0))
                    origin = " 初始化" if is_init else ""
                    print(
                        f"[轴{origin}] axis={number} raw={value:+6d} value={normalized:+.3f}",
                        flush=True,
                    )
            elif event_type == JS_EVENT_BUTTON and (show_init or not is_init):
                state = "按下" if value else "松开"
                origin = " 初始化" if is_init else ""
                print(f"[按钮{origin}] button={number} state={state} value={value}", flush=True)
    except KeyboardInterrupt:
        print("\n已退出，只读测试结束。")
        return 0
    except OSError as error:
        print(f"读取 {device} 失败: {error.strerror}", file=sys.stderr)
        return 1
    finally:
        os.close(fd)


def main() -> int:
    args = parse_args()
    return run(args.device, args.threshold, args.show_init)


if __name__ == "__main__":
    raise SystemExit(main())
