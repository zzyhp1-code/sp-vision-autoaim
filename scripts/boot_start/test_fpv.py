#!/usr/bin/env python3
"""Offline checks; no camera, vision process or real stream is started."""
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
import fpv_publisher as fpv


class FpvTests(unittest.TestCase):
    def test_only_exact_camera_window(self):
        tree = '''  0x12 "reprojection": ("standard" "Standard") 800x540
  0x13 "reprojection": ("terminal" "Terminal") 800x540
  0x14 "standard": ("standard" "Standard") 20x20
'''
        self.assertEqual(fpv.window_candidates(tree), ["0x12"])

    def test_window_pid_and_visibility(self):
        replies = [subprocess.CompletedProcess([], 0, '0x12 "reprojection": ("standard" "Standard")'),
                   subprocess.CompletedProcess([], 0, '_NET_WM_PID(CARDINAL) = 123'),
                   subprocess.CompletedProcess([], 0, 'Map State: IsViewable')]
        with patch.object(fpv.subprocess, "run", side_effect=replies), \
             patch.object(fpv.os, "readlink", return_value="/project/build/standard (deleted)"):
            self.assertEqual(fpv.camera_window({}, Path('/project/build/standard')), '0x12')

    def test_wrong_process_rejected(self):
        replies = [subprocess.CompletedProcess([], 0, '0x12 "reprojection": ("standard" "Standard")'),
                   subprocess.CompletedProcess([], 0, '_NET_WM_PID(CARDINAL) = 123')]
        with patch.object(fpv.subprocess, "run", side_effect=replies), \
             patch.object(fpv.os, "readlink", return_value="/other/standard"):
            self.assertIsNone(fpv.camera_window({}, Path('/project/build/standard')))

    def test_no_desktop_waits(self):
        with patch.object(fpv, "display_environment", return_value=None):
            self.assertIsNone(fpv.ready(Path('/project'))[1])

    def test_no_window_does_not_open_network(self):
        with patch.object(fpv, "display_environment", return_value={}), \
             patch.object(fpv, "camera_window", return_value=None), \
             patch.object(fpv.socket, "create_connection") as connect:
            self.assertIsNone(fpv.ready(Path('/project'))[1])
            connect.assert_not_called()

    def test_command_uses_window_with_stream_brightness(self):
        args = fpv.command('0x42')
        self.assertEqual(args[args.index('-window_id') + 1], '0x42')
        self.assertIn('eq=gamma=1.25', args[args.index('-vf') + 1])
        self.assertEqual(args[-1], fpv.URL)

    def test_supervisor_rebinds_after_window_changes(self):
        class Stop:
            count = 0
            def is_set(self):
                return self.count >= 4
            def wait(self, _):
                self.count += 1
                return self.is_set()
            def set(self):
                self.count = 4
        child = MagicMock()
        child.poll.return_value = None
        with patch.object(sys, 'argv', ['fpv_publisher.py']), \
             patch.object(fpv.Path, 'mkdir'), patch.object(fpv.Path, 'open'), \
             patch.object(fpv.fcntl, 'flock'), patch.object(fpv.signal, 'signal'), \
             patch.object(fpv.threading, 'Event', return_value=Stop()), \
             patch.object(fpv, 'existing_publisher', return_value=None), \
             patch.object(fpv, 'ready', side_effect=[({'DISPLAY': ':0'}, '0x12', 'ready'),
                                                    ({'DISPLAY': ':0'}, '0x34', 'ready')]), \
             patch.object(fpv, 'camera_window', return_value=None), \
             patch.object(fpv.subprocess, 'Popen', return_value=child) as launch, \
             patch.object(fpv, 'stop_child') as stop:
            fpv.main()
            self.assertEqual(launch.call_count, 2)
            self.assertEqual(stop.call_count, 2)
            for call, expected in zip(launch.call_args_list, ['0x12', '0x34']):
                args = call.args[0]
                self.assertEqual(args[args.index('-window_id') + 1], expected)


if __name__ == '__main__':
    unittest.main()
