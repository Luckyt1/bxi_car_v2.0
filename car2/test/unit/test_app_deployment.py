#!/usr/bin/env python3
"""Exercise App startup and OTA artifacts with fake executables, never hardware."""

import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('package_ota', ROOT / 'scripts/package_ota.py')
PACKAGER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PACKAGER)


class AppDeploymentTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='car2-app-deploy-')
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.prefix = self.directory / 'installed'
        config = self.prefix / 'share/chassis/config'
        config.mkdir(parents=True)
        for name in ('steering.yaml', 'steering_hardware.yaml', 'yiyou_ethercat.yaml'):
            (config / name).write_text(f'{name}: default\n')
        (config / 'yiyou_ethercat.yaml').write_text(
            'chassis:\n  ros__parameters:\n    yiyou_ethercat_slaves: [1, 2, 3, 4]\n')
        for name in ('LICENSE', 'vendor/soem/LICENSE', 'vendor/soem/ORIGIN.md',
                     'vendor/soem/LOCAL_PATCHES.md'):
            path = self.prefix / 'share/chassis' / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('license and source record\n')
        bin_dir = self.prefix / 'lib/chassis'
        bin_dir.mkdir(parents=True)
        for name in ('chassis', 'chassis_only'):
            binary = bin_dir / name
            binary.write_text(
                '#!/usr/bin/python3\n'
                'import json, os, pathlib, sys, time\n'
                'record = {"argv": sys.argv, "env": dict(os.environ), "cwd": os.getcwd()}\n'
                'pathlib.Path(os.environ["CAPTURE"]).write_text(json.dumps(record))\n'
                'if os.environ.get("HOLD"): time.sleep(60)\n'
            )
            binary.chmod(0o755)
        shutil.copytree(ROOT / 'scripts', self.prefix / 'scripts')
        self.ros_setup = self.directory / 'ros.bash'
        self.ros_setup.write_text('export ROS_DOMAIN_ID=123\nexport RMW_IMPLEMENTATION=setup_default\n')
        (self.prefix / 'local_setup.bash').write_text('export ROS_LOCALHOST_ONLY=1\n')
        self.config = self.directory / 'persistent/config'
        self.state = self.directory / 'persistent/state'
        self.lock = self.directory / 'persistent/control.lock'
        self.capture = self.directory / 'capture.json'
        self.environment = {
            **os.environ,
            'BXI_CAR2_ROOT': str(self.prefix),
            'BXI_CAR2_CONFIG_DIR': str(self.config),
            'BXI_CAR2_STATE_DIR': str(self.state),
            'BXI_CAR2_LOG_DIR': str(self.directory / 'logs'),
            'BXI_CAR2_LOCK_FILE': str(self.lock),
            'BXI_CAR2_ROS_SETUP': str(self.ros_setup),
            'ROS_DOMAIN_ID': '22',
            'ROS_LOCALHOST_ONLY': '0',
            'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
            'CYCLONEDDS_URI': '<CycloneDDS/>',
            'CAPTURE': str(self.capture),
            'PYTHONDONTWRITEBYTECODE': '1',
        }

    def run_script(self, name, *args, environment=None):
        return subprocess.run(
            ['bash', str(self.prefix / 'scripts' / name), *args],
            env=environment or self.environment,
            text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10,
            check=False,
        )

    def initialize(self):
        result = self.run_script('postinstall.sh')
        self.assertEqual(result.returncode, 0, result.stdout)

    def trusted_reference(self):
        # This fixture is consumed only by a fake executable, never a motor driver.
        (self.state / 'yiyou_motor4_position.yaml').write_text('test-only-position-reference\n')

    def test_dry_run_is_read_only_and_no_hardware_command_runs(self):
        result = self.run_script('run_app_control.sh', '--dry-run')
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('app_interface_enabled:=true', result.stdout)
        self.assertIn('ROS_DOMAIN_ID=22', result.stdout)
        self.assertFalse(self.lock.parent.exists())
        self.assertFalse(self.capture.exists())

    def test_postinstall_seeds_once_and_preserves_calibration_and_position(self):
        self.initialize()
        (self.config / 'steering_hardware.yaml').write_text('calibrated: true\n')
        (self.state / 'yiyou_motor4_position.yaml').write_text('position: 123456\n')
        (self.prefix / 'share/chassis/config/steering_hardware.yaml').write_text('replacement\n')
        self.initialize()
        self.assertEqual((self.config / 'steering_hardware.yaml').read_text(), 'calibrated: true\n')
        self.assertEqual((self.state / 'yiyou_motor4_position.yaml').read_text(), 'position: 123456\n')
        self.assertFalse(self.capture.exists())

    def test_migration_uses_existing_machine_before_new_defaults(self):
        old = self.directory / 'old-car2'
        (old / 'src/config').mkdir(parents=True)
        (old / 'state').mkdir()
        (old / 'src/config/steering_hardware.yaml').write_text('old-calibration\n')
        (old / 'state/yiyou_motor4_position.yaml').write_text('old-position\n')
        result = self.run_script('postinstall.sh', '--migrate-from', str(old))
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual((self.config / 'steering_hardware.yaml').read_text(), 'old-calibration\n')
        self.assertEqual((self.state / 'yiyou_motor4_position.yaml').read_text(), 'old-position\n')
        self.assertTrue((self.config / 'yiyou_ethercat.yaml').is_file())

    def test_postinstall_rejects_persistent_path_inside_package(self):
        environment = {**self.environment, 'BXI_CAR2_CONFIG_DIR': str(self.prefix / 'config')}
        result = self.run_script('postinstall.sh', environment=environment)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('outside OTA package', result.stdout)

    def test_start_preserves_dds_and_uses_external_config_and_position_memory(self):
        self.initialize()
        self.trusted_reference()
        result = self.run_script('run_app_control.sh')
        self.assertEqual(result.returncode, 0, result.stdout)
        captured = json.loads(self.capture.read_text())
        for name in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI'):
            self.assertEqual(captured['env'][name], self.environment[name])
        self.assertEqual(captured['env']['CHASSIS_CONFIG_DIR'], str(self.config))
        self.assertEqual(captured['cwd'], str(self.config))
        self.assertIn(f'yiyou_motor_4_position_memory_file:={self.state}/yiyou_motor4_position.yaml',
                      captured['argv'])
        self.assertIn('app_interface_enabled:=true', captured['argv'])
        self.assertNotIn('yiyou_arm_home_on_start:=false', captured['argv'])

    def test_chassis_only_omits_arm_configuration(self):
        self.initialize()
        environment = {**self.environment, 'BXI_CAR2_ENABLE_ARM': '0'}
        result = self.run_script('run_app_control.sh', environment=environment)
        self.assertEqual(result.returncode, 0, result.stdout)
        captured = json.loads(self.capture.read_text())
        self.assertEqual(Path(captured['argv'][0]).name, 'chassis_only')
        self.assertNotIn('--params-file', captured['argv'])

    def test_start_rejects_ota_lock(self):
        self.initialize()
        self.trusted_reference()
        with self.lock.open('w') as owner:
            fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
            result = self.run_script('run_app_control.sh')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('OTA deployment is active', result.stdout)
        self.assertFalse(self.capture.exists())

    def test_control_process_keeps_lock_until_exit(self):
        self.initialize()
        self.trusted_reference()
        process = subprocess.Popen(
            ['bash', str(self.prefix / 'scripts/run_app_control.sh')],
            env={**self.environment, 'HOLD': '1'},
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        try:
            deadline = time.monotonic() + 5
            while not self.capture.exists() and time.monotonic() < deadline and process.poll() is None:
                time.sleep(0.02)
            self.assertTrue(self.capture.exists())
            with self.lock.open('w') as contender:
                with self.assertRaises(BlockingIOError):
                    fcntl.flock(contender, fcntl.LOCK_EX | fcntl.LOCK_NB)
        finally:
            process.terminate()
            process.communicate(timeout=5)
        with self.lock.open('w') as contender:
            fcntl.flock(contender, fcntl.LOCK_EX | fcntl.LOCK_NB)

    def test_fourth_axis_requires_existing_trusted_reference_before_hardware(self):
        self.initialize()
        self.assertFalse((self.state / 'yiyou_motor4_position.yaml').exists())
        result = self.run_script('run_app_control.sh')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('requires a trusted position reference', result.stdout)
        self.assertFalse(self.capture.exists())

    def test_legacy_controller_without_lock_prevents_second_hardware_owner(self):
        self.initialize()
        self.trusted_reference()
        legacy = self.directory / 'legacy/chassis'
        legacy.parent.mkdir()
        shutil.copy2('/bin/sleep', legacy)
        process = subprocess.Popen([str(legacy), '60'])
        try:
            result = self.run_script('run_app_control.sh')
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('Another car2 controller is running', result.stdout)
            self.assertFalse(self.capture.exists())
        finally:
            process.terminate()
            process.wait(timeout=5)

    def test_three_axes_do_not_require_fourth_axis_position_memory(self):
        self.initialize()
        (self.config / 'yiyou_ethercat.yaml').write_text(
            'chassis:\n  ros__parameters:\n    yiyou_ethercat_slaves: [1, 2, 3]\n')
        result = self.run_script('run_app_control.sh')
        self.assertEqual(result.returncode, 0, result.stdout)

    def test_package_has_version_root_hash_licenses_and_excludes_runtime_data(self):
        (self.prefix / 'state').mkdir()
        (self.prefix / 'state/position.yaml').write_text('private-runtime')
        (self.prefix / 'log').mkdir()
        (self.prefix / 'log/can.log').write_text('runtime-log')
        archive, checksum = PACKAGER.package(self.prefix, self.directory / 'releases', 'v261030-1')
        self.assertEqual(checksum.read_text().split()[0], hashlib.sha256(archive.read_bytes()).hexdigest())
        with tarfile.open(archive) as bundle:
            names = bundle.getnames()
            self.assertTrue(all(name == 'car2' or name.startswith('car2/') for name in names))
            self.assertEqual(bundle.extractfile('car2/.version').read(), b'v261030-1\n')
            self.assertIn('car2/share/chassis/vendor/soem/LICENSE', names)
            self.assertFalse(any(name.startswith(('car2/state', 'car2/log')) for name in names))

    def test_package_rejects_development_symlinks(self):
        (self.prefix / 'source-link').symlink_to(ROOT / 'src')
        with self.assertRaisesRegex(ValueError, 'non-symlink install'):
            PACKAGER.package(self.prefix, self.directory / 'releases', 'v261030')

    def test_package_rejects_invalid_version_and_incomplete_install(self):
        with self.assertRaisesRegex(ValueError, 'Version'):
            PACKAGER.package(self.prefix, self.directory / 'releases', '../v261030')
        (self.prefix / 'lib/chassis/chassis').unlink()
        with self.assertRaisesRegex(ValueError, 'Incomplete install'):
            PACKAGER.package(self.prefix, self.directory / 'releases', 'v261030')


if __name__ == '__main__':
    unittest.main()
