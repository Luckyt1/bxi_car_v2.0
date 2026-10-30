#!/usr/bin/env bash
# Pixel preview by default; --joystick waits for button6, --tracking-mode depth uses depth.
set -eo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
vision_python="${CHASSIS_VISION_PYTHON:-$repo_root/.venv-vision/bin/python}"
if [[ -z "${CHASSIS_VISION_PYTHON:-}" && ! -x "$vision_python" ]]; then
  vision_python=python3
fi
ros_setup="${CHASSIS_ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ -f "$ros_setup" ]]; then
  source "$ros_setup"
fi
# Match deploy/ros_elf_launch.service; explicit caller settings take precedence.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-89}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
tracking_dds_default='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="lo" multicast="true"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-$tracking_dds_default}"
exec "$vision_python" -B "$repo_root/src/example/arm_tracking.py" "$@"
