#!/usr/bin/env python3

"""Exercise the real ROS entry points with a FIFO and an absent hardware config."""

import contextlib
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time
import unittest


EXECUTABLES = []


class RemoteProcess:

    def __init__(self, executable, directory, joystick=True, start_button=14):
        self.directory = Path(directory)
        self.device = self.directory / 'js0'
        self.config = self.directory / 'deliberately-absent.yaml'
        self.log_path = self.directory / 'output.log'
        self.fd = None
        if joystick:
            os.mkfifo(self.device)
            self.fd = os.open(self.device, os.O_RDWR | os.O_NONBLOCK)
        self.log_file = self.log_path.open('w')
        env = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=directory)
        self.proc = subprocess.Popen(
            [executable, '--ros-args',
             '-p', f'device_path:={self.device}',
             '-p', f'steering_config_file:={self.config}',
             '-p', f'start_button:={start_button}'],
            stdout=self.log_file, stderr=subprocess.STDOUT, env=env,
        )

    def output(self):
        return self.log_path.read_text()

    def wait_for(self, text):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if text in self.output():
                return
            if self.proc.poll() is not None:
                break
            time.sleep(0.02)
        raise AssertionError(f'Did not find {text!r}:\n{self.output()}')

    def button(self, number, pressed, initial=False):
        event = struct.pack('IhBB', 0, int(pressed), 0x01 | (0x80 if initial else 0), number)
        os.write(self.fd, event)

    def close(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=4)
        self.log_file.close()
        if self.fd is not None:
            os.close(self.fd)


@contextlib.contextmanager
def remote(executable, **kwargs):
    with tempfile.TemporaryDirectory(prefix='remote-start-cli-') as directory:
        process = RemoteProcess(executable, directory, **kwargs)
        try:
            yield process
        finally:
            process.close()


class RemoteStartCliTests(unittest.TestCase):

    def assert_waiting(self, process):
        time.sleep(0.25)
        self.assertIsNone(process.proc.poll(), process.output())
        self.assertNotIn('chassis startup requested', process.output())
        self.assertNotIn('bad file', process.output())
        self.assertNotIn('CAN0: calibrating', process.output())

    def test_both_entries_wait_without_a_joystick_or_config(self):
        for executable in EXECUTABLES:
            with self.subTest(executable=executable):
                with remote(executable, joystick=False) as process:
                    process.wait_for('Waiting for button14')
                    self.assert_waiting(process)

    def test_held_start_is_ignored_then_fresh_press_reaches_config_loading(self):
        with remote(EXECUTABLES[0]) as process:
            process.wait_for('Waiting for button14')
            process.button(11, False, initial=True)
            process.button(14, True, initial=True)
            process.button(14, True)
            self.assert_waiting(process)
            process.button(14, False)
            process.button(14, True)
            self.assertEqual(process.proc.wait(timeout=5), 1, process.output())
            self.assertEqual(process.output().count('chassis startup requested'), 1)
            self.assertIn('bad file', process.output())
            self.assertIn(str(process.config), process.output())

    def test_initial_emergency_stop_exits_before_hardware_startup(self):
        with remote(EXECUTABLES[0]) as process:
            process.wait_for('Waiting for button14')
            process.button(11, True, initial=True)
            self.assertEqual(process.proc.wait(timeout=5), 0, process.output())
            self.assertNotIn('bad file', process.output())
            self.assertNotIn('chassis startup requested', process.output())

    def test_emergency_in_start_batch_cancels_request_before_hardware_startup(self):
        with remote(EXECUTABLES[0]) as process:
            process.wait_for('Waiting for button14')
            events = [(11, False), (14, False), (14, True), (11, True)]
            batch = b''.join(struct.pack('IhBB', 0, int(v), 0x01, n) for n, v in events)
            os.write(process.fd, batch)
            self.assertEqual(process.proc.wait(timeout=5), 0, process.output())
            self.assertIn('chassis startup requested', process.output())
            self.assertNotIn('bad file', process.output())

    def test_configured_start_button_on_combined_entry(self):
        with remote(EXECUTABLES[1], start_button=12) as process:
            process.wait_for('Waiting for button12')
            process.button(11, False, initial=True)
            process.button(14, False)
            process.button(14, True)
            self.assert_waiting(process)
            process.button(12, False)
            process.button(12, True)
            self.assertEqual(process.proc.wait(timeout=5), 1, process.output())
            self.assertIn('button12 pressed: chassis startup requested', process.output())
            self.assertIn('bad file', process.output())


if __name__ == '__main__':
    EXECUTABLES = sys.argv[1:3]
    unittest.main(argv=[sys.argv[0]])
