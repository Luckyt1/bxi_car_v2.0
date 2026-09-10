#!/usr/bin/env bash
set -Eeo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$script_dir/diffbot_chassis"
executable="$workspace/install/chassis/lib/chassis/motor_power_hold"

if [[ ! -x "$executable" ]]; then
  echo "请先构建 diffbot_chassis 中的 chassis 包。" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"

echo "本程序只保持共享电机电源开启，不会使能或转动电机。"
echo "请先停止 chassis、steering_calibration 和其他 BXI 程序。"
echo "程序会持续运行以保持上电；关闭程序时自动断电。"
exec "$executable"
