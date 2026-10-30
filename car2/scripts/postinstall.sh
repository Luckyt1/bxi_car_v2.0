#!/usr/bin/env bash
# Trusted OTA hook: validate dependencies and seed persistent files, never start hardware.
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$script_dir/postinstall.py" "$@"
