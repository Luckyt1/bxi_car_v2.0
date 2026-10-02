"""Offline checks for menu routing and diagnostic executable argument boundaries."""

import contextlib
import importlib.util
import io
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


debug = load_module("debug_entry_test", ROOT / "test" / "debug.py")
yiyou = load_module("yiyou_entry_test", ROOT / "test" / "tools" / "yiyou_control.py")


class DebugMenuTests(unittest.TestCase):
    def run_entry(self, argv, inputs=()):
        output = io.StringIO()
        with mock.patch("builtins.input", side_effect=inputs), \
                mock.patch.object(debug.subprocess, "Popen") as run, \
                contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
            result = debug.main(argv)
        return result, output.getvalue(), run

    def test_every_numeric_menu_route_is_dry_run_only(self):
        routes = [(["1", "1"], "--brand bxi"), (["1", "2"], "yiyou_control.py jog"),
                  (["1", "3"], "--brand iswv"), (["2"], "arm_solver"),
                  (["3", "1"], "chassis_only --ros-args"),
                  (["3", "2"], "chassis_only_remote --ros-args")]
        for keys, expected in routes:
            with self.subTest(keys=keys):
                result, output, run = self.run_entry(["--dry-run", "--interface", "eth0"], keys)
                self.assertEqual(result, 0)
                self.assertIn(expected, output)
                run.assert_not_called()

    def test_invalid_menu_input_reprompts(self):
        result, output, run = self.run_entry(["--dry-run"], ["wrong", "1", "7", "1"])
        self.assertEqual(result, 0)
        self.assertEqual(output.count("输入无效"), 2)
        run.assert_not_called()

    def test_zero_eof_and_interrupt_exit_every_menu_level(self):
        for keys in [["0"], [EOFError], [KeyboardInterrupt], ["1", "0"],
                     ["1", EOFError], ["3", "0"], ["3", EOFError]]:
            with self.subTest(keys=keys):
                result, _, run = self.run_entry([], keys)
                self.assertEqual(result, 0)
                run.assert_not_called()

    def test_all_modes_accept_direct_dry_run(self):
        for mode in ["bxi", "yiyou", "iswv", "arm", "chassis", "chassis-remote"]:
            with self.subTest(mode=mode):
                result, _, run = self.run_entry(
                    ["--mode", mode, "--dry-run", "--interface", "eth0"])
                self.assertEqual(result, 0)
                run.assert_not_called()

    def test_invalid_motor_and_ethercat_parameters_do_not_spawn(self):
        arguments = [["--mode", "iswv", "--node", "128"],
                     ["--mode", "iswv", "--node", "0"],
                     ["--mode", "yiyou"],
                     ["--mode", "yiyou", "--interface", "lo"],
                     ["--mode", "yiyou", "--interface", "eth0", "--slaves", "1,1"],
                     ["--mode", "yiyou", "--interface", "eth0", "--slaves", "0"],
                     ["--mode", "yiyou", "--interface", "eth0", "--slaves", "200"],
                     ["--mode", "yiyou", "--interface", "eth0", "--slaves", "1,abc"]]
        for argv in arguments:
            with self.subTest(argv=argv):
                result, _, run = self.run_entry(argv + ["--dry-run"])
                self.assertEqual(result, 2)
                run.assert_not_called()

    def test_parser_rejects_unknown_options_bus_and_node_before_spawn(self):
        for argv in [["--bus", "7"], ["--bus", "-1"], ["--node", "2048"],
                     ["--node", "nan"], ["--mode", "unknown"], ["--typo"]]:
            with self.subTest(argv=argv), mock.patch.object(debug.subprocess, "Popen") as run, \
                    contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as raised:
                    debug.main(argv)
                self.assertEqual(raised.exception.code, 2)
                run.assert_not_called()

    def test_config_path_with_spaces_is_one_argument(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "steering file.yaml"
            config.write_text("{}")
            args = debug.arguments(["--dry-run", "--config", str(config)])
            command = debug.command_for("chassis", args)
            self.assertEqual(command[-1], "steering_config_file:=" + str(config))

    def test_explicit_bus_node_and_model_are_forwarded(self):
        args = debug.arguments(["--dry-run", "--bus", "5", "--node", "0x7ff", "--model", "8515"])
        command = debug.command_for("bxi", args)
        self.assertEqual(command[1:], ["--brand", "bxi", "--bus", "5", "--node", "2047",
                                       "--model", "8515"])

    def test_yiyou_home_uses_existing_frontend_and_backend(self):
        args = debug.arguments(["--dry-run", "--interface", "enp2s0", "--slaves", "1,199",
                                "--bus", "6", "--yiyou-mode", "home"])
        command = debug.command_for("yiyou", args)
        self.assertEqual(command[2:9], ["home", "--interface", "enp2s0", "--slaves", "1,199",
                                        "--power-bus", "6"])
        self.assertEqual(Path(command[-1]).name, "arm_yiyou_control")

    def test_missing_config_is_rejected(self):
        result, _, run = self.run_entry(
            ["--mode", "arm", "--dry-run", "--config", "/missing/config"])
        self.assertEqual(result, 2)
        run.assert_not_called()

    def test_actual_route_uses_argv_without_shell_and_propagates_failure(self):
        with mock.patch.object(debug, "resolve_tool", return_value=Path("/fake/motor_console")), \
                mock.patch.object(debug.subprocess, "Popen") as run, \
                contextlib.redirect_stdout(io.StringIO()):
            run.return_value.wait.return_value = 7
            self.assertEqual(debug.main(["--mode", "bxi"]), 7)
        run.assert_called_once_with(["/fake/motor_console", "--brand", "bxi", "--bus", "3",
                                     "--node", "1", "--model", "5014"], start_new_session=True)

    def test_help_and_eof_in_real_python_process_are_harmless(self):
        for arguments, stdin in [(["--help"], ""), ([], ""), ([], "0\n")]:
            result = subprocess.run([sys.executable, str(ROOT / "test" / "debug.py"), *arguments],
                                    input=stdin, text=True, capture_output=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)


class ToolResolutionTests(unittest.TestCase):
    def test_default_build_outputs_use_project_root(self):
        with mock.patch.dict(os.environ, {}, clear=True), \
                mock.patch.object(debug, "source_root", return_value=ROOT), \
                mock.patch.object(Path, "is_file", return_value=False):
            self.assertEqual(debug.resolve_tool("motor_console", dry_run=True),
                             ROOT / "build/chassis/motor_console")

    def test_explicit_directories_do_not_fall_back_to_project_defaults(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for folder in ["test", "src/thirdparty", "build/chassis",
                           "install/chassis/lib/chassis"]:
                (root / folder).mkdir(parents=True)
            entry = root / "test" / "debug.py"
            entry.touch()
            (root / "build/chassis/motor_console").touch()
            (root / "install/chassis/lib/chassis/motor_console").touch()
            build = root / "external-build"
            install = root / "external-install"
            with mock.patch.object(debug, "__file__", str(entry)), \
                    mock.patch.dict(os.environ, {"CHASSIS_BUILD_DIR": str(build),
                                                 "CHASSIS_INSTALL_DIR": str(install)}):
                self.assertEqual(debug.source_root(), root)
                self.assertEqual(debug.default_config("steering.yaml"),
                                 root / "src/config/steering.yaml")
                with self.assertRaises(ValueError):
                    debug.resolve_tool("motor_console")
                self.assertEqual(debug.resolve_tool("motor_console", dry_run=True),
                                 build / "chassis/motor_console")
                current = build / "chassis/motor_console"
                current.parent.mkdir(parents=True)
                current.touch()
                self.assertEqual(debug.resolve_tool("motor_console"), current)
                current.unlink()
                installed = install / "chassis/lib/chassis/motor_console"
                installed.parent.mkdir(parents=True)
                installed.touch()
                self.assertEqual(debug.resolve_tool("motor_console"), installed)

    def test_installed_siblings_win_even_when_source_ancestors_exist(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "src/thirdparty").mkdir(parents=True)
            (root / "test").mkdir()
            (root / "test/debug.py").touch()
            prefix = root / "external-install/chassis"
            tools = prefix / "lib/chassis"
            tools.mkdir(parents=True)
            (tools / "motor_console").touch()
            with mock.patch.object(debug, "__file__", str(tools / "debug.py")):
                self.assertIsNone(debug.source_root())
                self.assertEqual(debug.resolve_tool("motor_console"), tools / "motor_console")
                self.assertEqual(debug.default_config("steering.yaml"),
                                 prefix / "share/chassis/config/steering.yaml")

    def test_relocated_install_runs_without_source_or_build(self):
        with tempfile.TemporaryDirectory() as directory:
            prefix = Path(directory) / "relocated"
            tools = prefix / "lib/chassis"
            config = prefix / "share/chassis/config"
            tools.mkdir(parents=True)
            config.mkdir(parents=True)
            shutil.copy(ROOT / "test/debug.py", tools / "debug.py")
            shutil.copy(ROOT / "test/tools/yiyou_control.py", tools / "yiyou_control.py")
            (config / "steering.yaml").write_text("{}")
            for mode in ["bxi", "yiyou", "chassis-remote"]:
                result = subprocess.run([sys.executable, str(tools / "debug.py"), "--mode", mode,
                                         "--dry-run", "--interface", "eth0"],
                                        text=True, capture_output=True, timeout=5)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn(str(tools), result.stdout)
                self.assertNotIn("/tmp/car2.1-build", result.stdout)

    def test_yiyou_backend_search_uses_new_build_and_installed_sibling(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "test/tools/yiyou_control.py"
            source.parent.mkdir(parents=True)
            source.touch()
            build = root / "external-build"
            install = root / "external-install"
            backend = build / "chassis/arm_yiyou_control"
            backend.parent.mkdir(parents=True)
            backend.write_text("#!/bin/sh\nexit 0\n")
            backend.chmod(0o755)
            with mock.patch.object(yiyou, "__file__", str(source)), \
                    mock.patch.dict(os.environ, {"CHASSIS_BUILD_DIR": str(build),
                                                 "CHASSIS_INSTALL_DIR": str(install)}):
                self.assertEqual(yiyou.find_backend(), backend)
            default_backend = root / "build/chassis/arm_yiyou_control"
            default_backend.parent.mkdir(parents=True)
            shutil.copy(backend, default_backend)
            with mock.patch.object(yiyou, "__file__", str(source)), \
                    mock.patch.dict(os.environ, {}, clear=True):
                self.assertEqual(yiyou.find_backend(), default_backend)
            installed = install / "chassis/lib/chassis"
            installed.mkdir(parents=True)
            shutil.copy(backend, installed / "arm_yiyou_control")
            with mock.patch.object(yiyou, "__file__", str(installed / "yiyou_control.py")):
                self.assertEqual(yiyou.find_backend(), installed / "arm_yiyou_control")
                (installed / "arm_yiyou_control").unlink()
                with self.assertRaises(yiyou.UserError):
                    yiyou.find_backend()


class ShutdownTests(unittest.TestCase):
    def test_interrupts_allow_slow_cleanup_for_parent_and_terminal_group_signals(self):
        for signum, group in [(signal.SIGINT, False), (signal.SIGINT, True),
                              (signal.SIGTERM, False), (signal.SIGHUP, False)]:
            with self.subTest(signal=signum, terminal_group=group), \
                    tempfile.TemporaryDirectory() as directory:
                tools = Path(directory) / "lib/chassis"
                tools.mkdir(parents=True)
                shutil.copy(ROOT / "test/debug.py", tools / "debug.py")
                ready = Path(directory) / "ready"
                cleanup = Path(directory) / "cleanup"
                backend = tools / "motor_console"
                backend.write_text(
                    f"#!{sys.executable}\n"
                    "import os, signal, sys, time\n"
                    "from pathlib import Path\n"
                    "def stop(signum, frame):\n"
                    f"    if Path({str(cleanup)!r}).exists(): sys.exit(9)\n"
                    f"    Path({str(cleanup)!r}).write_text('started:' + str(signum))\n"
                    "    time.sleep(0.6)\n"
                    f"    Path({str(cleanup)!r}).write_text('done:' + str(signum))\n"
                    "    sys.exit(7)\n"
                    "for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):\n"
                    "    signal.signal(signum, stop)\n"
                    f"Path({str(ready)!r}).write_text(str(os.getpid()))\n"
                    "while True: time.sleep(0.1)\n")
                backend.chmod(0o755)
                process = subprocess.Popen(
                    [sys.executable, "-c",
                     "import runpy, signal, sys; "
                     "signal.signal(signal.SIGINT, signal.default_int_handler); "
                     "sys.argv = sys.argv[1:]; runpy.run_path(sys.argv[0], run_name='__main__')",
                     str(tools / "debug.py"), "--mode", "bxi"],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                    start_new_session=True)
                try:
                    deadline = time.monotonic() + 5
                    while (not ready.exists() and process.poll() is None and
                           time.monotonic() < deadline):
                        time.sleep(0.01)
                    self.assertTrue(ready.exists(), "stub must be ready before interruption")
                    if group:
                        os.killpg(process.pid, signum)
                    else:
                        process.send_signal(signum)
                    deadline = time.monotonic() + 3
                    while not cleanup.exists() and time.monotonic() < deadline:
                        time.sleep(0.01)
                    self.assertTrue(cleanup.exists(), "cleanup must start after forwarded signal")
                    # Repeated parent/terminal interrupts must not interrupt ongoing cleanup.
                    if group:
                        os.killpg(process.pid, signum)
                    else:
                        process.send_signal(signum)
                    stdout, stderr = process.communicate(timeout=5)
                    self.assertEqual(process.returncode, 7, stdout + stderr)
                    self.assertEqual(cleanup.read_text(), "done:" + str(int(signum)))
                finally:
                    if process.poll() is None:
                        process.kill()
                    if ready.exists():
                        try:
                            os.kill(int(ready.read_text()), signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                    process.communicate(timeout=5)


class NativeDiagnosticTests(unittest.TestCase):
    @staticmethod
    def executable(name):
        if "CHASSIS_TEST_BIN_DIR" in os.environ:
            executable = Path(os.environ["CHASSIS_TEST_BIN_DIR"]) / name
            if not executable.is_file():
                raise AssertionError(f"Required diagnostic was not built: {executable}")
            return executable
        build = Path(os.environ.get("CHASSIS_BUILD_DIR", ROOT / "build"))
        for folder in [build / "chassis", build]:
            if (folder / name).is_file():
                return folder / name
        raise unittest.SkipTest(f"{name} has not been built yet")

    def test_motor_console_help_dry_run_and_invalid_arguments(self):
        executable = self.executable("motor_console")
        cases = [(["--help"], 0),
                 (["--brand", "bxi", "--bus", "3", "--node", "1", "--dry-run"], 0),
                 (["--brand", "iswv", "--bus", "1", "--node", "127", "--dry-run"], 0),
                 (["--brand", "bxi", "--bus", "7", "--node", "1"], 2),
                 (["--brand", "iswv", "--bus", "1", "--node", "128"], 2),
                 (["--brand", "bxi", "--bus", "0", "--node", "2048"], 2),
                 (["--brand", "bxi", "--bus", "0", "--node", "1", "--model", "unknown"], 2)]
        for argv, expected in cases:
            with self.subTest(argv=argv):
                result = subprocess.run([str(executable), *argv], text=True,
                                        capture_output=True, timeout=3)
                self.assertEqual(result.returncode, expected, result.stderr)

    def test_arm_solver_computes_and_rejects_invalid_console_input(self):
        executable = self.executable("arm_solver")
        command = [str(executable), "--config", str(ROOT / "src/config/arm_kinematics.json")]
        result = subprocess.run(command,
                                input="bad\n1 0 0 0\n2 5 0 0\n0\n", text=True,
                                capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("输入无效", result.stderr)
        self.assertIn("x=", result.stdout)
        self.assertIn("无可行逆解", result.stdout)


if __name__ == "__main__":
    unittest.main()
