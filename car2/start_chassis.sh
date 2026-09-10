#!/usr/bin/env bash
set -Eeo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$script_dir/diffbot_chassis"
config="${CHASSIS_CONFIG:-$workspace/src/chassis/config/steering.yaml}"
if [[ ! -x "$workspace/install/chassis/lib/chassis/chassis" ]]; then
  echo "请先构建 diffbot_chassis 中的 chassis 包。" >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
exec "$workspace/install/chassis/lib/chassis/chassis" --ros-args \
  -p "steering_config_file:=$config" -p "post_calibration_rpm:=${1:-0.0}"
