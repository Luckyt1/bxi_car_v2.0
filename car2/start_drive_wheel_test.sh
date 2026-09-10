#!/usr/bin/env bash
set -Eeo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$script_dir/diffbot_chassis"
config="${CHASSIS_CONFIG:-$workspace/src/chassis/config/steering.yaml}"
executable="$workspace/install/chassis/lib/chassis/drive_wheel_test"

if [[ $# -gt 2 || ${1:-} == --help ]]; then
  echo "用法：sudo -E bash start_drive_wheel_test.sh [轮号 1..4] [RPM]"
  echo "未提供参数时，脚本会交互选择一个行进轮并输入前进速度。"
  exit "$(( $# > 2 ))"
fi
if [[ ! -x "$executable" ]]; then
  echo "请先构建 diffbot_chassis 中的 chassis 包。" >&2
  exit 1
fi
if [[ ! -f "$config" ]]; then
  echo "找不到配置文件：$config" >&2
  exit 1
fi

wheel=${1:-}
if [[ -z "$wheel" ]]; then
  echo "选择要测试的行进轮："
  echo "  1) 前左  CAN1  Node-ID 1"
  echo "  2) 后左  CAN1  Node-ID 2"
  echo "  3) 后右  CAN2  Node-ID 3"
  echo "  4) 前右  CAN2  Node-ID 4"
  read -r -p "请输入 1..4：" wheel
fi

rpm=${2:-}
if [[ -z "$rpm" ]]; then
  read -r -p "底盘前进方向输出轴 RPM [0.5]：" rpm
  rpm=${rpm:-0.5}
fi

source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"

echo "警告：所选行进轮将实际转动，请架空车辆并准备急停。"
echo "请先停止 chassis、steering_calibration 和其他 BXI 程序。"
exec "$executable" "$config" "$wheel" "$rpm"
