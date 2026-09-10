#!/usr/bin/env bash
set -Eeo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$script_dir/diffbot_chassis"
if [[ $# -gt 1 || ${1:-} == --help ]]; then
  echo "用法：bash start_motor_info.sh [节点号 1..127]；选择读取信息或持续转动测试。"
  exit "$(( $# > 1 ))"
fi
node_id=${1:-}
if [[ -z $node_id ]]; then
  read -r -p "电机节点号 [1]：" node_id
  node_id=${node_id:-1}
fi
if [[ ! $node_id =~ ^[0-9]{1,3}$ ]] || (( 10#$node_id < 1 || 10#$node_id > 127 )); then
  echo "错误：电机节点号必须为 1..127。" >&2
  exit 1
fi
echo "选择功能："
echo "  1) 读取电机信息（默认）"
echo "  2) 持续转动测试（Ctrl+C 停机）"
read -r -p "请输入 1 或 2 [1]：" selection
case "${selection:-1}" in
  1)
    executable=eyou_motor_info
    args=("$node_id")
    ;;
  2)
    executable=eyou_motor_test
    read -r -p "输出轴转速 RPM（范围 ±30，负数反转）[1]：" rpm
    args=("$node_id" "${rpm:-1}" --continuous)
    ;;
  *)
    echo "错误：请输入 1 或 2。" >&2
    exit 1
    ;;
esac
test -x "$workspace/install/chassis/lib/chassis/$executable" || {
  echo "错误：请先编译 chassis，未找到 $executable。" >&2
  exit 1
}
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
set -u

mkdir -p "$script_dir/logs"
log_file="$(mktemp "$script_dir/logs/motor_info_$(date +%Y%m%d_%H%M%S)_XXXXXX.log")"
ln -sfnT "$(basename -- "$log_file")" "$script_dir/logs/latest_motor_info.log"
echo "日志文件：$log_file"
echo "请先停止其他底盘/校准/BXI 程序；本程序会上电，退出时关闭共享电机电源。"
if [[ $executable == eyou_motor_test ]]; then
  echo "持续转动测试：切换 CANopen 速度模式并使能；按 Ctrl+C 减速停机并失能。"
else
  echo "只读取电机对象，不发送使能、转速、模式切换或清故障指令。"
fi
exec > >(trap '' INT TERM; exec tee -a "$log_file") 2>&1
# Run directly so the ROS launcher does not add an exit-code line after Ctrl+C.
exec stdbuf -oL -eL "$workspace/install/chassis/lib/chassis/$executable" "${args[@]}"
