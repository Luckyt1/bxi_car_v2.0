"""Offline launcher checks: temporary configurations, ROS setup and fake programs."""

import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
ROUTES = {"zero": ("start_yiyou_jog.sh", "yiyou_control.py"),
          "chassis": ("start_chassis_only.sh", "chassis_only_remote"),
          "robot": ("start_chassis_arm.sh", "robot_control")}


class StartScriptTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="car start scripts ")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        for folder in ["test/tools", "src/thirdparty", "src/config", "build/chassis",
                       "install", "ros", "other cwd"]:
            (self.root / folder).mkdir(parents=True, exist_ok=True)
        for name in ["start.sh", *(value[0] for value in ROUTES.values())]:
            shutil.copy2(ROOT / name, self.root / name)
        shutil.copy2(ROOT / "test/debug.py", self.root / "test/debug.py")
        for name in ["steering.yaml", "yiyou_ethercat.yaml"]:
            (self.root / "src/config" / name).write_text("{}\n")
        fake = ("#!/usr/bin/env python3\nimport json, os, pathlib, sys\n"
                "pathlib.Path(os.environ['FAKE_CAPTURE']).write_text(json.dumps(sys.argv))\n"
                "sys.exit(int(os.environ.get('FAKE_STATUS', '0')))\n")
        for name in ["arm_yiyou_control", "chassis_only_remote", "robot_control"]:
            binary = self.root / "build/chassis" / name
            binary.write_text(fake)
            binary.chmod(0o755)
        (self.root / "test/tools/yiyou_control.py").write_text(fake)
        setup = 'printf loaded >> "$FAKE_ENV_CAPTURE"\n'
        (self.root / "ros/setup.bash").write_text(setup)
        (self.root / "install/local_setup.bash").write_text(setup)
        self.capture = self.root / "executed.json"
        self.environment_capture = self.root / "environment-loaded"
        self.env = dict(os.environ, CHASSIS_BUILD_DIR=str(self.root / "build"),
                        CHASSIS_INSTALL_DIR=str(self.root / "install"),
                        CHASSIS_CONFIG=str(self.root / "src/config/steering.yaml"),
                        YIYOU_CONFIG=str(self.root / "src/config/yiyou_ethercat.yaml"),
                        FAKE_CAPTURE=str(self.capture), FAKE_STATUS="0",
                        FAKE_ENV_CAPTURE=str(self.environment_capture),
                        ROS_DISTRO=os.path.relpath(self.root / "ros", "/opt/ros"),
                        YIYOU_INTERFACE="enp1s0", YIYOU_SLAVES="1,2,3")
        self.env.pop("BASH_ENV", None)

    def launch(self, *arguments, entry="start.sh", stdin="", **environment):
        return subprocess.run(["bash", str(self.root / entry), *arguments],
                              cwd=self.root / "other cwd", env=dict(self.env, **environment),
                              input=stdin, text=True, capture_output=True, timeout=5)

    def command(self, result):
        self.assertEqual(result.returncode, 0, result.stderr)
        line = next(line for line in result.stdout.splitlines()
                    if line.startswith("执行命令："))
        return shlex.split(line.split("：", 1)[1])

    def assert_no_execution(self):
        self.assertFalse(self.capture.exists())
        self.assertFalse(self.environment_capture.exists())

    def test_numeric_menu_and_shortcuts_select_expected_programs(self):
        for number, (mode, (entry, program)) in enumerate(ROUTES.items(), 1):
            for result in [self.launch("--dry-run", stdin=f"{number}\n"),
                           self.launch("--dry-run", entry=entry)]:
                with self.subTest(mode=mode):
                    command = self.command(result)
                    index = 1 if mode == "zero" else 0
                    self.assertEqual(Path(command[index]).name, program)
                    if mode == "zero":
                        self.assertEqual(command[2], "jog")
        self.assert_no_execution()

    def test_default_root_build_and_install_work_from_another_directory(self):
        self.env.pop("CHASSIS_BUILD_DIR")
        self.env.pop("CHASSIS_INSTALL_DIR")
        preview = self.command(self.launch("robot", "--dry-run"))
        self.assertEqual(Path(preview[0]), self.root / "build/chassis/robot_control")
        result = self.launch("robot")
        self.assertEqual(result.returncode, 0, result.stderr)
        command = json.loads(self.capture.read_text())
        self.assertEqual(Path(command[0]), self.root / "build/chassis/robot_control")
        self.assertEqual(self.environment_capture.read_text(), "loaded" * 2)

    def test_help_and_preview_do_not_load_environment_or_execute_programs(self):
        for mode, (entry, _) in ROUTES.items():
            for arguments in [("--help",), ("--dry-run",)]:
                result = self.launch(*arguments, entry=entry, ROS_DISTRO="missing-offline-test")
                self.assertEqual(result.returncode, 0, result.stderr)
        self.assert_no_execution()
        shutil.rmtree(self.root / "build")
        shutil.rmtree(self.root / "src/config")
        self.assertEqual(self.launch("--help").returncode, 0)

    def test_menu_reprompts_and_exit_or_eof_are_harmless(self):
        self.assertIn("请输入", self.launch("--dry-run", stdin="wrong\n2\n").stdout)
        for stdin in ["", "0\n"]:
            self.assertEqual(self.launch(stdin=stdin).returncode, 0)
        self.assertEqual(self.launch("unknown").returncode, 2)
        self.assert_no_execution()

    def test_configs_and_extra_arguments_preserve_spaces_from_other_directory(self):
        config = self.root / "steering override.yaml"
        arm = self.root / "arm override.yaml"
        config.write_text("{}")
        arm.write_text("{}")
        extra = ["--ros-args", "-p", "example:=a b $(never_execute)"]
        command = self.command(self.launch("robot", "--dry-run", *extra,
                                           CHASSIS_CONFIG=str(config), YIYOU_CONFIG=str(arm)))
        self.assertIn("steering_config_file:=" + str(config), command)
        self.assertEqual(command[command.index("--params-file") + 1], str(arm))
        self.assertEqual(command[-len(extra):], extra)
        self.assert_no_execution()

    def test_missing_programs_and_configs_fail_before_execution(self):
        for mode, name in [("zero", "arm_yiyou_control"),
                           ("chassis", "chassis_only_remote"), ("robot", "robot_control")]:
            (self.root / "build/chassis" / name).unlink()
            self.assertNotEqual(self.launch(mode).returncode, 0)
        for mode, key in [("chassis", "CHASSIS_CONFIG"), ("robot", "YIYOU_CONFIG")]:
            result = self.launch(mode, "--dry-run", **{key: str(self.root / "missing.yaml")})
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("配置文件不存在", result.stderr)
        self.assert_no_execution()

    def test_zero_exec_preserves_arguments_and_exit_status(self):
        extra = ["--calibration-file", str(self.root / "my zero.json"), "--slaves", "2"]
        result = self.launch(*extra, entry="start_yiyou_jog.sh", FAKE_STATUS="37")
        self.assertEqual(result.returncode, 37, result.stderr)
        command = json.loads(self.capture.read_text())
        self.assertEqual(command[1], "jog")
        self.assertEqual(command[-len(extra):], extra)
        self.assertFalse(self.environment_capture.exists())

    @unittest.skipUnless(Path("/opt/ros").is_dir(), "ROS prefix needed for fake setup path")
    def test_ros_routes_exec_fake_program_and_propagate_exit_status(self):
        for mode in ["chassis", "robot"]:
            result = self.launch(mode, "--ros-args", "-p", "test:=a b", FAKE_STATUS="23")
            self.assertEqual(result.returncode, 23, result.stderr)
            command = json.loads(self.capture.read_text())
            self.assertEqual(Path(command[0]).name, ROUTES[mode][1])
            self.assertEqual(command[-3:], ["--ros-args", "-p", "test:=a b"])
        self.assertEqual(self.environment_capture.read_text(), "loaded" * 4)


if __name__ == "__main__":
    unittest.main()
