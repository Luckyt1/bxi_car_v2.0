#!/usr/bin/env python3
"""Seed persistent car2 settings without replacing machine calibration or state."""

import argparse
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess


def copy_missing(source, target):
    """Only create absent regular files; preserve an existing destination verbatim."""
    for path in sorted(source.rglob('*')):
        if path.is_symlink():
            raise ValueError(f'Migration/default files must not be symlinks: {path}')
        if not path.is_file():
            continue
        destination = target / path.relative_to(source)
        destination.parent.mkdir(parents=True, exist_ok=True)
        try:
            with destination.open('xb') as output, path.open('rb') as original:
                shutil.copyfileobj(original, output)
        except FileExistsError:
            continue
        except Exception:
            destination.unlink(missing_ok=True)
            raise
        os.chmod(destination, 0o644)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--migrate-from', type=Path,
                        help='Existing car2 source tree; copy src/config and state only if absent')
    args = parser.parse_args()
    root = Path(os.environ.get('BXI_CAR2_ROOT') or Path(__file__).resolve().parent.parent)
    config = Path(os.environ.get('BXI_CAR2_CONFIG_DIR', '/var/lib/bxi/car2/config'))
    state = Path(os.environ.get('BXI_CAR2_STATE_DIR', '/var/lib/bxi/car2/state'))
    logs = Path(os.environ.get('BXI_CAR2_LOG_DIR', '/var/log/bxi/car2'))
    lock = Path(os.environ.get('BXI_CAR2_LOCK_FILE', '/var/lib/bxi/car2/control.lock'))
    defaults = root / 'share/chassis/config'
    for path in (root, config, state, logs, lock):
        if not path.is_absolute():
            raise ValueError(f'car2 paths must be absolute: {path}')
    # A future OTA directory swap must never contain the live configuration.
    for path in (config, state, logs, lock):
        if path.resolve().is_relative_to(root.resolve()):
            raise ValueError(f'Persistent path must be outside OTA package: {path}')
    for name in ('steering.yaml', 'steering_hardware.yaml', 'yiyou_ethercat.yaml'):
        if not (defaults / name).is_file():
            raise ValueError(f'Missing installed configuration: {defaults / name}')
    for name in ('chassis', 'chassis_only'):
        binary = root / 'lib/chassis' / name
        if not os.access(binary, os.X_OK):
            raise ValueError(f'Missing executable: {binary}')
    ros_setup = Path(os.environ.get('BXI_CAR2_ROS_SETUP', '/opt/ros/humble/setup.bash'))
    if not ros_setup.is_file():
        raise ValueError(f'ROS 2 Humble is required: {ros_setup}')
    if importlib.util.find_spec('yaml') is None:
        raise ValueError('python3-yaml from the ROS 2 Humble runtime is required for startup preflight')
    setup = root / 'local_setup.bash'
    if not setup.is_file():
        setup = root / 'share/chassis/local_setup.bash'
    if not setup.is_file():
        raise ValueError(f'Missing installed ROS environment: {setup}')
    for name in ('chassis', 'chassis_only'):
        binary = root / 'lib/chassis' / name
        with binary.open('rb') as stream:
            is_elf = stream.read(4) == b'\x7fELF'
        if is_elf:
            result = subprocess.run(
                ['bash', '-c', 'source "$1" && export AMENT_CURRENT_PREFIX="$2" && '
                 'source "$3" && ldd "$4"', 'car2-dependencies', str(ros_setup),
                 str(root), str(setup), str(binary)],
                text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False,
            )
            if result.returncode or 'not found' in result.stdout:
                raise ValueError(f'Missing car2 runtime dependencies (ROS Humble / yaml-cpp):\n{result.stdout}')
    migration = args.migrate_from
    if migration is not None and not (migration / 'src/config').is_dir():
        raise ValueError(f'Missing migration configuration: {migration / "src/config"}')
    for path in (config, state, logs, lock.parent):
        path.mkdir(parents=True, exist_ok=True)
    if migration is not None:
        # Copy the old machine first. New defaults only fill newly added settings files.
        copy_missing(migration / 'src/config', config)
        if (migration / 'state').is_dir():
            copy_missing(migration / 'state', state)
    copy_missing(defaults, config)
    print(f'car2 persistent config: {config}')
    print(f'car2 persistent state: {state}; existing files preserved')
    if not (state / 'yiyou_motor4_position.yaml').is_file():
        print('Fourth-axis startup is blocked until a trusted position reference exists. '
              'Use postinstall.sh --migrate-from SOURCE to copy this machine\'s existing state; '
              'otherwise establish a reference using the existing calibration workflow. '
              'No default position memory was created.')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError) as error:
        raise SystemExit(f'car2 postinstall: {error}') from error
