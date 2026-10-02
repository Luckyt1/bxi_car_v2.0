#!/usr/bin/env python3
"""Exercise installed/build ROS entrypoints without initializing any motor hardware."""

import argparse
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class RosContract(unittest.TestCase):
    bin_dir = Path()
    config = Path()

    def run_entry(self, name, *arguments):
        if name == "chassis" and arguments != ("--help",):
            # Stop before hardware even if an arm parameter override regresses.
            arguments += ("--", "--ros-args", "-p", "can3_motor_1_angle_deg:=1.0")
        with tempfile.TemporaryDirectory(prefix="chassis-ros-contract-") as log_dir:
            environment = os.environ.copy()
            environment.update(ROS_LOCALHOST_ONLY="1", ROS_LOG_DIR=log_dir)
            return subprocess.run(
                [str(self.bin_dir / name), *arguments],
                env=environment,
                cwd=log_dir,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=8,
                check=False,
            )

    def reject_startup(self, name, parameters, expected):
        arguments = ["--ros-args"]
        for parameter in parameters:
            arguments.extend(["-p", parameter])
        result = self.run_entry(name, *arguments)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn(expected, result.stdout)

    def test_help_preserves_alias_modes_and_parameters(self):
        modes = {
            "chassis": "chassis",
            "key_control": "combined",
            "robot_control": "combined",
            "chassis_only": "chassis_only",
            "chassis_only_remote": "chassis_only_remote",
        }
        for entry, mode in modes.items():
            with self.subTest(entry=entry):
                result = self.run_entry(entry, "--help")
                self.assertEqual(result.returncode, 0, result.stdout)
                self.assertIn(f"Mode: {mode}\n", result.stdout)
                self.assertIn("--ros-args", result.stdout)
                self.assertIn("steering_config_file", result.stdout)
                self.assertIn("start_button:=14", result.stdout)
                self.assertIn("emergency_stop_button:=11", result.stdout)
                self.assertIn("Help does not open CAN devices", result.stdout)
                if entry in {"chassis", "key_control", "robot_control"}:
                    self.assertIn("yiyou_ethercat_slaves", result.stdout)
                    self.assertIn("yiyou_arm_target", result.stdout)
                    self.assertIn("/arm_pose", result.stdout)
                else:
                    self.assertIn("Chassis only: CAN0/1/2", result.stdout)

    def test_default_steering_config_works_outside_project_directory(self):
        for entry in ("chassis", "chassis_only"):
            with self.subTest(entry=entry):
                self.reject_startup(
                    entry, ["post_calibration_rpm:=-1.0"],
                    "post_calibration_rpm must be in [0, drive_max_output_rpm]",
                )

    def test_invalid_command_timeout_is_rejected_before_hardware(self):
        for value in ("0.0", "-0.1", ".nan", ".inf"):
            with self.subTest(value=value):
                self.reject_startup(
                    "chassis_only", [f"command_timeout_sec:={value}"],
                    "command_timeout_sec must be positive and finite",
                )

    def test_default_arm_config_is_loaded_before_motion_validation(self):
        self.reject_startup(
            "chassis", ["yiyou_arm_speed_rpm:=-1.0"],
            "invalid arm profile speed/acceleration",
        )

    def test_explicit_steering_config_overrides_default(self):
        with tempfile.TemporaryDirectory() as directory:
            missing = Path(directory) / "custom-steering.yaml"
            self.reject_startup(
                "chassis_only", [f"steering_config_file:={missing}",
                                 "post_calibration_rpm:=-1.0"], str(missing),
            )

    def test_missing_parameter_file_is_reported_without_aborting(self):
        with tempfile.TemporaryDirectory() as directory:
            missing = Path(directory) / "missing.yaml"
            result = self.run_entry("chassis", "--ros-args", "--params-file", str(missing))
            self.assertEqual(result.returncode, 1, result.stdout)
            self.assertIn(str(missing), result.stdout)

    def test_explicit_parameter_file_overrides_default_arm_config(self):
        with tempfile.TemporaryDirectory() as directory:
            parameters = Path(directory) / "arm.yaml"
            parameters.write_text(
                "chassis:\n  ros__parameters:\n"
                "    yiyou_ethercat_interface: lo\n"
                "    yiyou_arm_speed_rpm: -1.0\n")
            result = self.run_entry("chassis", "--ros-args", "--params-file", str(parameters))
            self.assertEqual(result.returncode, 1, result.stdout)
            self.assertIn("specify a physical EtherCAT --interface", result.stdout)

    def test_partial_parameter_file_keeps_defaults_and_cli_overrides(self):
        with tempfile.TemporaryDirectory() as directory:
            parameters = Path(directory) / "arm.yaml"
            parameters.write_text(
                "chassis:\n  ros__parameters:\n    yiyou_arm_speed_rpm: -1.0\n")
            result = self.run_entry("chassis", "--ros-args", "--params-file", str(parameters))
            self.assertEqual(result.returncode, 1, result.stdout)
            self.assertIn("invalid arm profile speed/acceleration", result.stdout)
            result = self.run_entry(
                "chassis", "--ros-args", "--params-file", str(parameters),
                "-p", "yiyou_ethercat_interface:=lo")
            self.assertEqual(result.returncode, 1, result.stdout)
            self.assertIn("specify a physical EtherCAT --interface", result.stdout)

    def test_negative_startup_speed_is_rejected(self):
        self.reject_startup(
            "chassis_only",
            [f"steering_config_file:={self.config}", "post_calibration_rpm:=-1.0"],
            "post_calibration_rpm must be in [0, drive_max_output_rpm]",
        )

    def test_duplicate_arm_slaves_are_rejected(self):
        self.reject_startup(
            "chassis",
            [f"steering_config_file:={self.config}", "yiyou_ethercat_slaves:=[1,1,3]"],
            "set three or four unique yiyou_ethercat_slaves",
        )

    def test_loopback_ethercat_is_rejected(self):
        self.reject_startup(
            "chassis",
            [f"steering_config_file:={self.config}", "yiyou_ethercat_interface:=lo"],
            "specify a physical EtherCAT --interface",
        )

    def test_node_specific_override_takes_precedence_over_default_file(self):
        self.reject_startup(
            "chassis", ["chassis:yiyou_ethercat_interface:=lo"],
            "specify a physical EtherCAT --interface",
        )

    def test_reserved_remote_axis_is_rejected(self):
        self.reject_startup("robot_control", ["axis_linear:=7"], "axis7 is reserved for the arm")

    def test_remote_speed_defaults_are_loaded_from_selected_steering_file(self):
        for parameter in ("scale_linear", "scale_lateral", "scale_angular",
                          "max_accel", "max_decel"):
            with self.subTest(parameter=parameter), tempfile.TemporaryDirectory() as directory:
                config = Path(directory) / "steering.yaml"
                config.write_text(f"remote_ctrl:\n  ros__parameters:\n    {parameter}: invalid\n")
                # The CLI guard rejects before joystick/hardware startup if loading regresses.
                self.reject_startup(
                    "chassis_only_remote",
                    [f"steering_config_file:={config}", "scale_linear:=-1.0"],
                    f"remote configuration parameter {parameter}")

    def test_remote_acceleration_defaults_are_applied(self):
        for parameter in ("max_accel", "max_decel"):
            with self.subTest(parameter=parameter), tempfile.TemporaryDirectory() as directory:
                config = Path(directory) / "steering.yaml"
                config.write_text(f"remote_ctrl:\n  ros__parameters:\n    {parameter}: -1.0\n")
                self.reject_startup(
                    "chassis_only_remote",
                    [f"steering_config_file:={config}", "scale_linear:=-1.0"],
                    "invalid joystick deadzone, low-pass time constant, timeout or acceleration")

    def test_remote_cli_speed_override_takes_precedence_over_steering_file(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "steering.yaml"
            config.write_text("remote_ctrl:\n  ros__parameters:\n"
                              "    max_accel: -1.0\n    scale_linear: -1.0\n")
            self.reject_startup(
                "chassis_only_remote", [f"steering_config_file:={config}", "max_accel:=1.5",
                                        "scale_lateral:=-1.0"],
                "invalid remote kinematics scale")

    def test_remote_ros_parameter_file_overrides_steering_defaults(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "steering.yaml"
            config.write_text("remote_ctrl:\n  ros__parameters:\n    max_accel: -1.0\n")
            overrides = Path(directory) / "overrides.yaml"
            overrides.write_text("remote_ctrl:\n  ros__parameters:\n    max_accel: 1.5\n")
            result = self.run_entry(
                "chassis_only_remote", "--ros-args", "--params-file", str(overrides),
                "-p", f"steering_config_file:={config}", "-p", "scale_linear:=-1.0")
            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn("invalid remote kinematics scale", result.stdout)

    def test_remote_steering_file_without_remote_section_keeps_parameter_defaults(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "steering.yaml"
            config.write_text("chassis:\n  ros__parameters: {}\n")
            self.reject_startup(
                "chassis_only_remote", [f"steering_config_file:={config}", "scale_linear:=-1.0"],
                "invalid remote kinematics scale")

    def test_remote_empty_steering_path_keeps_parameter_defaults(self):
        self.reject_startup(
            "chassis_only_remote", ["steering_config_file:=''", "scale_linear:=-1.0"],
            "invalid remote kinematics scale")

    def test_remote_missing_steering_file_is_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            missing = Path(directory) / "missing-steering.yaml"
            self.reject_startup(
                "chassis_only_remote", [f"steering_config_file:={missing}", "scale_linear:=-1.0"],
                str(missing))

    def test_remote_malformed_configuration_is_reported(self):
        documents = ["[", "[]\n", "remote_ctrl: []\n", "remote_ctrl:\n  ros__parameters: []\n",
                     "remote_ctrl:\n  ros__parameters:\n    scale_linear: [1, 2]\n"]
        for document in documents:
            with self.subTest(document=document), tempfile.TemporaryDirectory() as directory:
                config = Path(directory) / "steering.yaml"
                config.write_text(document)
                self.reject_startup(
                    "chassis_only_remote",
                    [f"steering_config_file:={config}", "scale_linear:=-1.0"],
                    "remote configuration")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bin-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    options, remaining = parser.parse_known_args()
    RosContract.bin_dir = options.bin_dir.resolve(strict=True)
    RosContract.config = options.config.resolve(strict=True)
    unittest.main(argv=[__file__, *remaining])
