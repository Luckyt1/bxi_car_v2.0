#!/usr/bin/env bash
# 日常启动入口；--dry-run 只显示命令，--help 不加载环境或访问硬件。
set -eo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<'HELP'
用法：
  sudo -E bash start.sh                         # 数字菜单
  sudo -E bash start.sh zero [意优工具参数]       # 点动并设置零位
  sudo -E bash start.sh chassis [ROS 参数]       # 仅底盘，等待手柄启动
  sudo -E bash start.sh robot [ROS 参数]         # 底盘 + 机械臂，等待手柄启动
  bash start.sh robot --dry-run                 # 只预览命令

意优：默认网口 enp86s0、站位 1,2,3；可用 --interface / --slaves 覆盖。
      左右选轴，上下点动，z 保存当前轴 EEPROM 零位，q 退出。
      启动工具本身不会设零；设零会保存该轴全部可持久化参数。
底盘/整机：松开后按 button14 启动，button11 急停，Ctrl-C 退出。
      整机按配置先返回已保存的机械臂零位，再校准底盘；不会重新设零。

环境变量：
  CHASSIS_BUILD_DIR     构建目录，默认项目根目录下 build
  CHASSIS_INSTALL_DIR   安装目录，默认项目根目录下 install
  CHASSIS_CONFIG       底盘 YAML，默认 src/config/steering.yaml
  YIYOU_CONFIG         整机机械臂 YAML，默认 src/config/yiyou_ethercat.yaml
  YIYOU_INTERFACE / YIYOU_SLAVES  设零工具网口/站位的默认值
  JOYSTICK_DEVICE      手柄设备，默认 /dev/input/js0
  JOYSTICK_START_BUTTON / JOYSTICK_ESTOP_BUTTON  启动/急停键，默认 14/11

其余参数原样传给对应程序；额外 ROS 参数放在末尾，例如：
  bash start.sh robot --dry-run --ros-args -p yiyou_arm_home_on_start:=false
脚本不自动提权、不修改网卡状态；实际运行需具备设备权限，EtherCAT 网口需已启用。
HELP
}

mode=""
dry_run=false
extra=()
if [[ "${1:-}" == zero || "${1:-}" == chassis || "${1:-}" == robot ]]; then
  mode="$1"
  shift
fi
for argument in "$@"; do
  case "$argument" in
    --dry-run) dry_run=true ;;
    -h|--help) usage; exit 0 ;;
    *) extra+=("$argument") ;;
  esac
done

if [[ -z "$mode" ]]; then
  if (( ${#extra[@]} )); then
    echo "未知启动模式：${extra[0]}；请使用 zero、chassis、robot 或 --help。" >&2
    exit 2
  fi
  while [[ -z "$mode" ]]; do
    printf '\n1 意优点动 / 设置零位\n2 底盘单独启动（手柄）\n3 整机启动（手柄）\n0 退出\n'
    read -r -p "选择：" choice || exit 0
    case "$choice" in
      1) mode=zero ;;
      2) mode=chassis ;;
      3) mode=robot ;;
      0) exit 0 ;;
      *) echo "请输入 0、1、2 或 3。" ;;
    esac
  done
fi

# 与调试菜单共用查找规则，默认使用项目根目录的 colcon 构建产物。
resolve_tool() {
  python3 -B - "$repo_root" "$1" "$dry_run" <<'PYTHON'
import sys
sys.path.insert(0, sys.argv[1] + "/test")
import debug
try:
    print(debug.resolve_tool(sys.argv[2], dry_run=sys.argv[3] == "true"))
except ValueError as error:
    print(error, file=sys.stderr)
    raise SystemExit(1)
PYTHON
}

if [[ "$mode" == zero ]]; then
  backend="$(resolve_tool arm_yiyou_control)"
  command=(python3 "$repo_root/test/tools/yiyou_control.py" jog
    --interface "${YIYOU_INTERFACE:-enp86s0}" --slaves "${YIYOU_SLAVES:-1,2,3}"
    --backend "$backend"
    --calibration-file "$repo_root/yiyou_ethercat_zero_calibration.json")
  echo "意优设零：左右选轴，上下点动，按 z 保存当前轴零位，q 退出。"
else
  binary=chassis_only_remote
  [[ "$mode" != robot ]] || binary=robot_control
  executable="$(resolve_tool "$binary")"
  config="${CHASSIS_CONFIG:-$repo_root/src/config/steering.yaml}"
  if [[ ! -f "$config" ]]; then
    echo "底盘配置文件不存在：$config" >&2
    exit 1
  fi
  command=("$executable" --ros-args)
  if [[ "$mode" == robot ]]; then
    yiyou_config="${YIYOU_CONFIG:-$repo_root/src/config/yiyou_ethercat.yaml}"
    if [[ ! -f "$yiyou_config" ]]; then
      echo "机械臂配置文件不存在：$yiyou_config" >&2
      exit 1
    fi
    command+=(--params-file "$yiyou_config")
    echo "整机启动：按启动键后依配置回到机械臂已保存零位，再校准底盘。"
  else
    echo "底盘单独启动：只控制底盘，不初始化意优机械臂。"
  fi
  command+=(-p "steering_config_file:=$config"
    -p post_calibration_rpm:=0.0 -p start_chassis:=true
    -p "device_path:=${JOYSTICK_DEVICE:-/dev/input/js0}"
    -p "start_button:=${JOYSTICK_START_BUTTON:-14}"
    -p "emergency_stop_button:=${JOYSTICK_ESTOP_BUTTON:-11}")
  echo "松开后按启动键 ${JOYSTICK_START_BUTTON:-14}；急停键 ${JOYSTICK_ESTOP_BUTTON:-11}；Ctrl-C 退出。"
fi
command+=("${extra[@]}")
printf '执行命令：'
printf ' %q' "${command[@]}"
printf '\n'
if [[ "$dry_run" == true ]]; then
  exit 0
fi

if [[ "$mode" != zero ]]; then
  ros_setup="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  if [[ ! -f "$ros_setup" ]]; then
    echo "找不到 ROS 环境：$ros_setup" >&2
    exit 1
  fi
  source "$ros_setup"
  install_setup="${CHASSIS_INSTALL_DIR:-$repo_root/install}/local_setup.bash"
  if [[ -f "$install_setup" ]]; then
    source "$install_setup"
  fi
fi
exec "${command[@]}"
