#!/usr/bin/env bash
set -Eeo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$script_dir/diffbot_chassis"
config="${CHASSIS_CONFIG:-$workspace/src/chassis/config/steering.yaml}"
chassis_bin="$workspace/install/chassis/lib/chassis/chassis"
remote_bin="$workspace/install/chassis/lib/chassis/key_control"

if [[ ! -x "$chassis_bin" || ! -x "$remote_bin" ]]; then
  echo "请先构建 diffbot_chassis 中的 chassis 包。" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"

chassis_pid=""
remote_pid=""
cleanup() {
  trap - EXIT INT TERM HUP
  [[ -n "$remote_pid" ]] && kill "$remote_pid" 2>/dev/null || true
  [[ -n "$chassis_pid" ]] && kill "$chassis_pid" 2>/dev/null || true
  [[ -n "$remote_pid" ]] && wait "$remote_pid" 2>/dev/null || true
  [[ -n "$chassis_pid" ]] && wait "$chassis_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM HUP

"$chassis_bin" --ros-args \
  -p "steering_config_file:=$config" \
  -p post_calibration_rpm:=0.0 &
chassis_pid=$!

"$remote_bin" --ros-args \
  -p "device_path:=${JOYSTICK_DEVICE:-/dev/input/js0}" \
  -p "axis_linear:=${JOYSTICK_LINEAR_AXIS:-3}" \
  -p "axis_angular:=${JOYSTICK_ANGULAR_AXIS:-6}" \
  -p "scale_linear:=${JOYSTICK_LINEAR_SCALE:-0.6}" \
  -p "scale_angular:=${JOYSTICK_ANGULAR_SCALE:-0.4}" \
  -p "invert_linear_axis:=${JOYSTICK_INVERT_LINEAR:-true}" \
  -p "invert_angular_axis:=${JOYSTICK_INVERT_ANGULAR:-true}" \
  -p "deadzone:=${JOYSTICK_DEADZONE:-0.05}" \
  -p "emergency_stop_button:=${JOYSTICK_ESTOP_BUTTON:-11}" \
  -p timeout_sec:=0.0 &
remote_pid=$!

status=0
wait -n "$chassis_pid" "$remote_pid" || status=$?
exit "$status"
