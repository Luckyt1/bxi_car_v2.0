#!/usr/bin/env bash
# Standalone camera recognition; no ROS environment or motor startup is required.
set -eo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
vision_python="${CHASSIS_VISION_PYTHON:-$repo_root/.venv-vision/bin/python}"
if [[ -z "${CHASSIS_VISION_PYTHON:-}" && ! -x "$vision_python" ]]; then
  vision_python=python3
fi
exec "$vision_python" -B "$repo_root/src/example/qr_midpoint.py" "$@"
