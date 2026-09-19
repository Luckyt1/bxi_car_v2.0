#!/usr/bin/env bash
set -Eeo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
log_root="${CHASSIS_LOG_DIR:-$script_dir/log/runtime}"
mkdir -p -- "$log_root"
run_dir="$(mktemp -d "$log_root/remote-control-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
log_file="$run_dir/console.log"
: > "$log_file"
ln -sfnT -- "$(basename -- "$run_dir")" "$log_root/latest"
# 同时保存 ROS 日志和厂商库直接写到 stdout/stderr 的 PCI/CAN 错误。
exec 3>&1 4>&2
exec > >(trap '' INT TERM HUP; exec tee -a -- "$log_file") 2>&1
logger_pid=$!

chassis_pid=""
remote_pid=""
cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP
  [[ -n "$remote_pid" ]] && kill "$remote_pid" 2>/dev/null || true
  [[ -n "$chassis_pid" ]] && kill "$chassis_pid" 2>/dev/null || true
  [[ -n "$remote_pid" ]] && wait "$remote_pid" 2>/dev/null || true
  [[ -n "$chassis_pid" ]] && wait "$chassis_pid" 2>/dev/null || true
  printf '\n[launcher] stopped=%s exit_status=%s\n' "$(date -Is)" "$status"
  exec 1>&3 2>&4 3>&- 4>&-
  wait "$logger_pid" || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

printf '[launcher] started=%s host=%s uid=%s\n' "$(date -Is)" "$(hostname)" "$(id -u)"
printf '[launcher] script=%s\n[launcher] log=%s\n' "$script_dir/start_remote_control.sh" "$log_file"
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

printf '[launcher] chassis=%s\n[launcher] remote=%s\n[launcher] config=%s\n' \
  "$(readlink -f -- "$chassis_bin")" "$(readlink -f -- "$remote_bin")" "$config"
sha256sum -- "$chassis_bin" "$remote_bin" "$config"
cp -- "$config" "$run_dir/steering-at-start.yaml"
if LC_ALL=C grep -aqF 'CAN3: save-zero commands sent' "$chassis_bin"; then
  echo '[launcher] CAN3 startup marker present in chassis binary (not device acknowledgement)'
else
  echo '[launcher] WARNING: CAN3 startup marker missing; check whether this is an old chassis binary'
fi

printf '[launcher] CAN3: Kp=200 Kd=4; abs(speed)>=60 RPM cuts shared motor power\n'
printf '[launcher] CAN3 remote: button3=previous, button1=next; axis7 adjusts 1 degree per deflection; axis6=chassis yaw\n'

stdbuf -oL -eL "$chassis_bin" --ros-args \
  -p "steering_config_file:=$config" \
  -p post_calibration_rpm:=0.0 &
chassis_pid=$!

stdbuf -oL -eL "$remote_bin" --ros-args \
  -p start_chassis:=false \
  -p "device_path:=${JOYSTICK_DEVICE:-/dev/input/js0}" \
  -p "axis_linear:=${JOYSTICK_LINEAR_AXIS:-3}" \
  -p "axis_lateral:=${JOYSTICK_LATERAL_AXIS:-0}" \
  -p "axis_angular:=${JOYSTICK_ANGULAR_AXIS:-6}" \
  -p "invert_can3_direction:=${JOYSTICK_CAN3_INVERT_DIRECTION:-false}" \
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
