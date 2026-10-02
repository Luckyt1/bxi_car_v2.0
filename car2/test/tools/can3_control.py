#!/usr/bin/env python3
"""Compatibility entry for the Yiyou EtherCAT jog terminal."""

import sys

from yiyou_control import main


if __name__ == '__main__':
    print('旧 CAN3 入口已迁移：请使用 debug.py --mode yiyou 或内部工具 yiyou_control.py jog。',
          file=sys.stderr)
    raise SystemExit(main(['jog', *sys.argv[1:]]))
