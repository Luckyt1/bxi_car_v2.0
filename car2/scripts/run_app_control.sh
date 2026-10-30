#!/usr/bin/env bash
# App-managed entry: a single hardware owner, with config/state outside OTA files.
set -eo pipefail

usage() {
  cat <<'HELP'
Usage: run_app_control.sh [--dry-run] [--ros-args ...]
Starts car2 without the joystick. --dry-run only prints the command.
BXI_CAR2_ROOT             installed prefix (default: this script's parent)
BXI_CAR2_ENABLE_ARM       1 (default) or 0
BXI_CAR2_CONFIG_DIR       /var/lib/bxi/car2/config
BXI_CAR2_STATE_DIR        /var/lib/bxi/car2/state
BXI_CAR2_LOG_DIR          /var/log/bxi/car2
BXI_CAR2_LOCK_FILE        /var/lib/bxi/car2/control.lock
DDS settings are inherited from the gateway; this script does not set a domain.
HELP
}

dry_run=false
extra=()
for argument in "$@"; do
  case "$argument" in
    --dry-run) dry_run=true ;;
    -h|--help) usage; exit 0 ;;
    *) extra+=("$argument") ;;
  esac
done

root="${BXI_CAR2_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
config_dir="${BXI_CAR2_CONFIG_DIR:-/var/lib/bxi/car2/config}"
state_dir="${BXI_CAR2_STATE_DIR:-/var/lib/bxi/car2/state}"
log_dir="${BXI_CAR2_LOG_DIR:-/var/log/bxi/car2}"
lock_file="${BXI_CAR2_LOCK_FILE:-/var/lib/bxi/car2/control.lock}"
ros_setup="${BXI_CAR2_ROS_SETUP:-/opt/ros/humble/setup.bash}"
enable_arm="${BXI_CAR2_ENABLE_ARM:-1}"
case "$enable_arm" in
  1) binary="$root/lib/chassis/chassis" ;;
  0) binary="$root/lib/chassis/chassis_only" ;;
  *) echo 'BXI_CAR2_ENABLE_ARM must be 0 or 1' >&2; exit 2 ;;
esac
for path in "$root" "$config_dir" "$state_dir" "$log_dir" "$lock_file"; do
  [[ "$path" == /* ]] || { echo "car2 paths must be absolute: $path" >&2; exit 2; }
done
canonical_root="$(realpath -m -- "$root")"
for path in "$config_dir" "$state_dir" "$log_dir" "$lock_file"; do
  case "$(realpath -m -- "$path")/" in
    "$canonical_root/"*) echo "Persistent path must be outside OTA package: $path" >&2; exit 2 ;;
  esac
done

command=("$binary" --ros-args -p "steering_config_file:=$config_dir/steering.yaml"
  -p app_interface_enabled:=true)
if [[ "$enable_arm" == 1 ]]; then
  command+=(--params-file "$config_dir/yiyou_ethercat.yaml"
    -p "yiyou_motor_4_position_memory_file:=$state_dir/yiyou_motor4_position.yaml")
fi
command+=("${extra[@]}")

if $dry_run; then
  printf 'CHASSIS_CONFIG_DIR=%q\n' "$config_dir"
  printf 'ROS_DOMAIN_ID=%q (inherited)\n' "${ROS_DOMAIN_ID:-unset}"
  printf 'lock=%q\n' "$lock_file"
  printf '%q ' "${command[@]}"
  printf '\n'
  exit 0
fi

[[ -x "$binary" ]] || { echo "Missing car2 executable: $binary" >&2; exit 1; }
[[ -r "$ros_setup" ]] || { echo "Missing ROS 2 Humble environment: $ros_setup" >&2; exit 1; }
for filename in steering.yaml steering_hardware.yaml; do
  [[ -r "$config_dir/$filename" ]] || {
    echo "Missing $config_dir/$filename; run scripts/postinstall.sh first" >&2; exit 1;
  }
done
if [[ "$enable_arm" == 1 && ! -r "$config_dir/yiyou_ethercat.yaml" ]]; then
  echo "Missing $config_dir/yiyou_ethercat.yaml; run scripts/postinstall.sh first" >&2
  exit 1
fi
if [[ "$enable_arm" == 1 ]]; then
  /usr/bin/python3 - "$config_dir/yiyou_ethercat.yaml" "$state_dir/yiyou_motor4_position.yaml" <<'PYTHON'
import pathlib
import sys

try:
    import yaml
except ImportError:
    raise SystemExit('car2 requires python3-yaml from the ROS 2 Humble runtime')
try:
    document = yaml.safe_load(pathlib.Path(sys.argv[1]).read_text())
    parameters = document['chassis']['ros__parameters']
    slaves = parameters.get('yiyou_ethercat_slaves', [1, 2, 3, 4])
    if not isinstance(slaves, list) or len(slaves) not in (3, 4):
        raise ValueError('yiyou_ethercat_slaves must select three or four axes')
    if len(slaves) == 4 and not pathlib.Path(sys.argv[2]).is_file():
        raise ValueError(
            'Fourth axis requires a trusted position reference: ' + sys.argv[2] +
            '; migrate the existing state using postinstall.sh --migrate-from SOURCE. '
            'If none exists, establish the reference using the existing calibration workflow; '
            'do not create an invented zero position.')
except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
    raise SystemExit('car2 startup preflight: ' + str(error))
PYTHON
fi

# Setups may contain defaults; preserve the gateway's exact DDS selection.
dds_names=(ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE)
dds_values=()
dds_present=()
for name in "${dds_names[@]}"; do
  dds_values+=("${!name}")
  [[ -v "$name" ]] && dds_present+=(1) || dds_present+=(0)
done
source "$ros_setup"
export AMENT_CURRENT_PREFIX="$root"
if [[ -r "$root/local_setup.bash" ]]; then
  source "$root/local_setup.bash"
elif [[ -r "$root/share/chassis/local_setup.bash" ]]; then
  source "$root/share/chassis/local_setup.bash"
else
  echo "Missing installed ROS environment in $root" >&2
  exit 1
fi
for index in "${!dds_names[@]}"; do
  if [[ "${dds_present[$index]}" == 1 ]]; then
    export "${dds_names[$index]}=${dds_values[$index]}"
  else
    unset "${dds_names[$index]}"
  fi
done

if [[ "$(head -c 4 "$binary")" == $'\177ELF' ]]; then
  dependencies="$(ldd "$binary" 2>&1)" || { echo "$dependencies" >&2; exit 1; }
  if [[ "$dependencies" == *'not found'* ]]; then
    echo "Missing car2 runtime libraries (ROS Humble / yaml-cpp): $dependencies" >&2
    exit 1
  fi
fi

mkdir -p -- "$(dirname -- "$lock_file")" "$state_dir" "$log_dir"
command -v flock >/dev/null || { echo 'car2 requires flock (util-linux)' >&2; exit 1; }
exec 9>"$lock_file"
flock -n 9 || { echo 'car2 is running or an OTA deployment is active; retry after it stops' >&2; exit 1; }
# A legacy direct launch does not hold this lock. Reject it instead of claiming
# the CAN/EtherCAT devices a second time. The OTA manager performs the same check.
for process in /proc/[0-9]*/exe; do
  executable="$(readlink -- "$process" 2>/dev/null || true)"
  executable="${executable% (deleted)}"
  case "${executable##*/}" in
    chassis|chassis_only|robot_control|key_control|chassis_only_remote)
      echo "Another car2 controller is running: $process ($executable)" >&2; exit 1 ;;
  esac
done

export CHASSIS_CONFIG_DIR="$config_dir"
export CHASSIS_CAN_LOG_DIR="$log_dir/can"
export CHASSIS_ARM_TORQUE_LOG_DIR="$log_dir/arm_torque"
# Relative model paths in the migrated YAML resolve beside the YAML files.
cd -- "$config_dir"
exec "${command[@]}"
