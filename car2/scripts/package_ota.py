#!/usr/bin/env python3
"""Create a car2 OTA archive from a completed, non-symlink ROS install prefix."""

import argparse
import hashlib
import io
from pathlib import Path
import re
import tarfile


def package(prefix, output, version):
    if re.fullmatch(r'v\d{6}(?:-[1-9]\d*)?', version) is None:
        raise ValueError('Version must be vYYMMDD or vYYMMDD-N')
    prefix = prefix.resolve(strict=True)
    required = (
        'lib/chassis/chassis', 'lib/chassis/chassis_only',
        'scripts/run_app_control.sh', 'scripts/postinstall.sh', 'scripts/postinstall.py',
        'share/chassis/config/steering.yaml', 'share/chassis/config/steering_hardware.yaml',
        'share/chassis/config/yiyou_ethercat.yaml', 'share/chassis/LICENSE',
        'share/chassis/vendor/soem/LICENSE', 'share/chassis/vendor/soem/ORIGIN.md',
        'share/chassis/vendor/soem/LOCAL_PATCHES.md',
    )
    for name in required:
        if not (prefix / name).is_file():
            raise ValueError(f'Incomplete install prefix: missing {name}')
    if not any((prefix / name).is_file() for name in
               ('local_setup.bash', 'share/chassis/local_setup.bash')):
        raise ValueError('Incomplete install prefix: missing local_setup.bash')
    # Never follow build-tree symlinks into an OTA artifact.
    excluded = {'build', 'install', 'log', 'logs', 'state', '.git', '.omx', '__pycache__'}
    paths = []
    for path in sorted(prefix.rglob('*')):
        relative = path.relative_to(prefix)
        if any(part in excluded for part in relative.parts) or relative == Path('.version'):
            continue
        if path.is_symlink():
            raise ValueError(f'OTA requires a non-symlink install: {relative}')
        if not path.is_dir() and not path.is_file():
            raise ValueError(f'Unsupported package file: {relative}')
        paths.append(path)
    archive = output / f'car2_{version}.tar.gz'
    if archive.resolve().is_relative_to(prefix):
        raise ValueError('Output directory must be outside the install prefix')
    if archive.exists() or archive.with_name(archive.name + '.sha256').exists():
        raise ValueError(f'Release already exists: {archive}; choose a new version/output directory')
    output.mkdir(parents=True, exist_ok=True)
    try:
        with tarfile.open(archive, 'w:gz', format=tarfile.PAX_FORMAT) as bundle:
            directory = tarfile.TarInfo('car2')
            directory.type = tarfile.DIRTYPE
            directory.mode = 0o755
            bundle.addfile(directory)
            for path in paths:
                info = bundle.gettarinfo(str(path), arcname=f'car2/{path.relative_to(prefix)}')
                info.uid = info.gid = 0
                info.uname = info.gname = 'root'
                if path.is_file():
                    with path.open('rb') as stream:
                        bundle.addfile(info, stream)
                else:
                    bundle.addfile(info)
            data = (version + '\n').encode()
            metadata = tarfile.TarInfo('car2/.version')
            metadata.size = len(data)
            metadata.mode = 0o644
            bundle.addfile(metadata, io.BytesIO(data))
        digest = hashlib.sha256()
        with archive.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        checksum = archive.with_name(archive.name + '.sha256')
        checksum.write_text(f'{digest.hexdigest()}  {archive.name}\n', encoding='utf-8')
    except Exception:
        archive.unlink(missing_ok=True)
        raise
    return archive, checksum


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install-prefix', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--version', required=True)
    args = parser.parse_args()
    try:
        for path in package(args.install_prefix, args.output_dir, args.version):
            print(path)
    except (OSError, ValueError) as error:
        parser.exit(1, f'car2 package: {error}\n')


if __name__ == '__main__':
    main()
