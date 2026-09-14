#!/usr/bin/env bash
set -Eeo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$script_dir/diffbot_chassis"
config="${CHASSIS_CONFIG:-$workspace/src/chassis/config/steering.yaml}"
install_dir=""
# 支持从 car2 根目录或 diffbot_chassis 子目录执行 colcon build。
for candidate in "$script_dir/install" "$workspace/install"; do
  if [[ -f "$candidate/setup.bash" &&
    -x "$candidate/chassis/lib/chassis/chassis" &&
    -x "$candidate/chassis/lib/chassis/key_control" ]]; then
    install_dir="$candidate"
    break
  fi
done
if [[ -z "$install_dir" ]]; then
  echo "未找到 chassis 和 key_control；请在 $script_dir 执行 colcon build --packages-select chassis。" >&2
  exit 1
fi
chassis_bin="$install_dir/chassis/lib/chassis/chassis"
remote_bin="$install_dir/chassis/lib/chassis/key_control"

source /opt/ros/humble/setup.bash
source "$install_dir/setup.bash"

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
  -p "axis_lateral:=${JOYSTICK_LATERAL_AXIS:-0}" \
  -p "axis_angular:=${JOYSTICK_ANGULAR_AXIS:-6}" \
  -p "scale_linear:=${JOYSTICK_LINEAR_SCALE:-0.6}" \
  -p "scale_lateral:=${JOYSTICK_LATERAL_SCALE:-0.6}" \
  -p "scale_angular:=${JOYSTICK_ANGULAR_SCALE:-0.4}" \
  -p "invert_linear_axis:=${JOYSTICK_INVERT_LINEAR:-true}" \
  -p "invert_lateral_axis:=${JOYSTICK_INVERT_LATERAL:-true}" \
  -p "invert_angular_axis:=${JOYSTICK_INVERT_ANGULAR:-true}" \
  -p "deadzone:=${JOYSTICK_DEADZONE:-0.05}" \
  -p "joystick_low_pass_time_constant_s:=${JOYSTICK_LOW_PASS_TIME_CONSTANT_S:-0.1}" \
  -p "emergency_stop_button:=${JOYSTICK_ESTOP_BUTTON:-11}" \
  -p timeout_sec:=0.0 &
remote_pid=$!

status=0
wait -n "$chassis_pid" "$remote_pid" || status=$?
exit "$status"
