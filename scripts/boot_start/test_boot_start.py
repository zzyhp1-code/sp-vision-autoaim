#!/usr/bin/env python3
"""Offline fixtures only: never run the real standard executable or open a device."""

import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


boot = module("boot_start", ROOT / "tools/boot_start_vision.py")
installer = module("boot_install", ROOT / "scripts/boot_start/install.py")


class BootTests(unittest.TestCase):
    def test_runtime_only_changes_color(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "configs").mkdir()
            original = 'enemy_color: "blue"\n# keep this comment\nxuc_fire_cadence_fallback_ms: 1100\n'
            config = root / "configs/standard.yaml"
            config.write_text(original)
            state = root / "state/sp-vision"
            state.mkdir(parents=True)
            (state / "field_autoaim_enemy_color").write_text("red\n")
            with patch.dict(os.environ, {"XDG_CONFIG_HOME": str(root / "state")}):
                _, color, runtime = boot.configuration(root, root)
            self.assertEqual(color, "red")
            self.assertEqual(runtime, original.replace('"blue"', '"red"'))
            self.assertEqual(config.read_text(), original)

    def test_bad_saved_color_falls_back_to_formal_config(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "configs").mkdir()
            (root / "configs/standard.yaml").write_text("enemy_color: blue\n")
            state = root / "state/sp-vision"
            state.mkdir(parents=True)
            (state / "field_autoaim_enemy_color").write_text("invalid")
            with patch.dict(os.environ, {"XDG_CONFIG_HOME": str(root / "state")}):
                _, color, _ = boot.configuration(root, root)
            self.assertEqual(color, "blue")

    def test_clean_environment(self):
        dirty = {key: "bad" for key in ["LD_LIBRARY_PATH", "LD_PRELOAD", "PYTHONPATH", "BASH_ENV", "ENV"]}
        dirty["HOME"] = "/home/fixture"
        clean = boot.clean_environment(dirty)
        self.assertEqual(clean["HOME"], dirty["HOME"])
        self.assertTrue(all(key not in clean for key in dirty if key != "HOME"))

    def test_atomic_write_does_not_call_mv(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "cache/runtime.yaml"
            with patch.object(boot.subprocess, "run", side_effect=AssertionError("no shell utilities")):
                boot.atomic_write(path, "first")
                boot.atomic_write(path, "second")
            self.assertEqual(path.read_text(), "second")
            self.assertEqual(list(path.parent.glob("*.tmp")), [])

    def test_check_never_starts_program_or_writes_cache(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            with patch.object(sys, "argv", ["boot", "--project", str(root), "--check"]), \
                 patch.object(boot, "configuration", return_value=({}, "blue", "cache")), \
                 patch.object(boot, "static_check"), \
                 patch.object(boot, "readiness", return_value=({}, "ready")), \
                 patch.object(boot, "supervise") as start:
                self.assertEqual(boot.main(), 0)
                start.assert_not_called()
            self.assertEqual(list(root.iterdir()), [])

    def test_existing_process_prevents_duplicate_start(self):
        with tempfile.TemporaryDirectory() as tmp:
            with patch.object(sys, "argv", ["boot", "--project", tmp]), \
                 patch.object(boot, "configuration", return_value=({}, "blue", "cache")), \
                 patch.object(boot, "static_check"), patch.object(boot.signal, "signal"), \
                 patch.object(boot, "existing_standard", return_value=1234), \
                 patch.object(boot, "supervise") as start:
                self.assertEqual(boot.main(), 0)
                start.assert_not_called()

    def test_waits_for_devices_before_start(self):
        with tempfile.TemporaryDirectory() as tmp:
            stop = threading.Event()
            with patch.object(sys, "argv", ["boot", "--project", tmp]), \
                 patch.object(boot, "configuration", return_value=({}, "blue", "cache")), \
                 patch.object(boot, "static_check"), patch.object(boot.signal, "signal"), \
                 patch.object(boot, "existing_standard", return_value=None), \
                 patch.object(boot.threading, "Event", return_value=stop), \
                 patch.object(stop, "wait", return_value=False), \
                 patch.object(boot, "readiness", side_effect=[(None, "waiting"), ({}, "ready")]) as ready, \
                 patch.object(boot, "supervise", return_value=0) as start:
                self.assertEqual(boot.main(), 0)
                self.assertEqual(ready.call_count, 2)
                start.assert_called_once()

    def test_supervised_dummy_stop_preserves_unrelated_process(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            dummy = root / "dummy.py"
            dummy.write_text('''import signal, time
signal.signal(signal.SIGINT, lambda *_: exit(0))
print('fixture running', flush=True)
while True: time.sleep(.05)
''')
            sentinel = subprocess.Popen(["sleep", "30"])
            stop = threading.Event()
            timer = threading.Timer(.5, stop.set)
            timer.start()
            try:
                with patch.object(boot, "runtime_command", return_value=[sys.executable, str(dummy)]):
                    self.assertEqual(boot.supervise(root, "blue", "enemy_color: blue\n", boot.clean_environment(), stop), 0)
                self.assertIsNone(sentinel.poll())
                runtime = root / "logs/xuc-bringup"
                self.assertFalse((runtime / "field_autoaim.pid").exists())
                log = Path((runtime / "field_autoaim.last_log").read_text().strip())
                self.assertIn("fixture running", log.read_text())
            finally:
                timer.cancel()
                sentinel.terminate()
                sentinel.wait()

    def test_failed_dummy_returns_failure_for_service_restart(self):
        with tempfile.TemporaryDirectory() as tmp:
            with patch.object(boot, "runtime_command", return_value=[sys.executable, "-c", "raise SystemExit(7)"]):
                self.assertEqual(boot.supervise(Path(tmp), "blue", "enemy_color: blue\n",
                                               boot.clean_environment(), threading.Event()), 1)

    def test_install_enables_only_does_not_start(self):
        with tempfile.TemporaryDirectory() as tmp:
            with patch.object(installer.Path, "home", return_value=Path(tmp)), \
                 patch.object(installer.subprocess, "run") as run:
                installer.main()
            commands = [call.args[0] for call in run.call_args_list]
            self.assertIn(["systemctl", "--user", "enable", "sp-vision-boot.service"], commands)
            self.assertTrue(all("start" not in args and "--now" not in args for args in commands))
            text = (Path(tmp) / ".config/systemd/user/sp-vision-boot.service").read_text()
            self.assertIn("WantedBy=default.target", text)
            self.assertIn("UnsetEnvironment=LD_PRELOAD LD_LIBRARY_PATH", text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
