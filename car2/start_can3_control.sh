#!/usr/bin/env bash
set -Eeo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$script_dir/diffbot_chassis"
script="$workspace/src/chassis/scripts/can3_control.py"
install_dir=""

if [[ ! -f "$script" ]]; then
  echo "未找到 CAN3 控制脚本: $script" >&2
  exit 1
fi

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  exec python3 "$script" "$@"
fi

for candidate in "$script_dir/install" "$workspace/install"; do
  if [[ -f "$candidate/setup.bash" &&
    -x "$candidate/chassis/lib/chassis/chassis" &&
    -x "$candidate/chassis/lib/chassis/key_control" ]]; then
    install_dir="$candidate"
    break
  fi
done

if [[ -z "$install_dir" ]]; then
  echo "未找到 chassis 和 key_control；请先构建 chassis 工作区。" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$install_dir/setup.bash"

exec python3 "$script" "$@"
