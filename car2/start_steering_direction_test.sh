#!/usr/bin/env bash
set -Eeo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$script_dir/diffbot_chassis"
config="${CHASSIS_CONFIG:-$workspace/src/chassis/config/steering.yaml}"
executable="$workspace/build/chassis/steering_calibration"

if [[ ${1:-} == --help ]]; then
  echo "用法：sudo -E bash start_steering_direction_test.sh"
  echo "同一次上电：两两校准、四轮回中，再依次前左/后左/后右/前右正向 +50° 点动。"
  echo "每轮输入 1 回车，以输出轴 2 RPM 点动 +50°，随后输入 1=顺时针、2=逆时针、0=未动、3=动错轮、q=退出；方向配置不修改。"
  exit 0
fi
if [[ $# -ne 0 || ! -x "$executable" || ! -f "$config" ]]; then
  echo "请先构建 steering_calibration，并确认配置文件存在；本脚本不接受位置参数。" >&2
  exit 1
fi

echo "先停止 chassis、校准、上电保持及其他电机程序。"
echo "警告：本流程会实际搜索机械限位！请架空固定车辆，检查限位，准备独立硬件急停。"
echo "校准将更新中位数据，沿用现场搜索速度和力矩阈值，不修改 inverted。"
grep -E '^[[:space:]]*(calibration_search_speed_rpm|calibration_contact_torque_nm|calibration_torque_limit_nm):' "$config"
echo "四轮回中后不重启电源，按前左、后左、后右、前右逐个正向点动 +50°。"
echo "观察统一从车顶向下看；等待回答期间转向电机仍上电保持位置。"
read -r -p "确认安全条件满足，输入 CALIBRATE 开始（其他输入取消）：" confirmation
[[ "$confirmation" == CALIBRATE ]] || { echo "已取消，未上电。"; exit 0; }

source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
exec "$executable" --ros-args -p "steering_config_file:=$config" \
  -p hold_after_calibration:=false -p confirm_directions:=true
