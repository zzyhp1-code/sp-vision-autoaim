"""Offline regression checks; all writes stay in temporary directories."""
import argparse
import contextlib
import importlib.util
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("yaml_tool", ROOT / "scripts/debug_menu/yaml_tool.py")
YAML_TOOL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(YAML_TOOL)


class MaintenanceTests(unittest.TestCase):
    def test_crc_command_exit_status(self):
        script = ROOT / "scripts/analysis/hipnuc_crc_check.py"
        original = (ROOT / "firmware/infantry/tests/hipnuc_crc_test.cpp").read_text(encoding="utf-8")
        cases = {
            "valid": (original, True),
            "bad_crc": (original.replace("0x02, 0x9D, 0x91", "0x02, 0x9C, 0x91", 1), False),
            "truncated": (original.replace("0xE7, 0xB2, 0x9B, 0x3D", "0xE7, 0xB2, 0x9B", 1), False),
            "missing_array": ("no sample", False),
            "invalid_byte": (original.replace("0x5A, 0xA5", "0x15A, 0xA5", 1), False),
        }
        with tempfile.TemporaryDirectory() as directory:
            sample = Path(directory) / "sample.cpp"
            commands = [([], False), ([str(Path(directory) / "missing.cpp")], False)]
            for name, (text, valid) in cases.items():
                with self.subTest(name=name):
                    sample.write_text(text, encoding="utf-8")
                    self.check_command(script, [str(sample)], valid)
            for arguments, valid in commands:
                with self.subTest(arguments=arguments):
                    self.check_command(script, arguments, valid)

    def check_command(self, script, arguments, valid):
        result = subprocess.run([sys.executable, str(script), *arguments],
                                capture_output=True, encoding="utf-8",
                                env={**os.environ, "PYTHONIOENCODING": "utf-8"})
        self.assertEqual(result.returncode == 0, valid, result.stdout + result.stderr)
        self.assertNotIn("Traceback", result.stderr)

    def test_pair_write_and_rollback(self):
        for failed_rollbacks in (None, (), ("a",), ("b",), ("a", "b")):
            with self.subTest(failed_rollbacks=failed_rollbacks), tempfile.TemporaryDirectory() as directory:
                folder = Path(directory)
                targets = [folder / "a.yaml", folder / "b.yaml"]
                candidate = folder / "candidate.yaml"
                candidate.write_text("value: 3\n", encoding="utf-8")
                originals = [b"# first\nvalue: 1\n", b"# second\nvalue: 2\n"]
                for target, original in zip(targets, originals):
                    target.write_bytes(original)
                args = argparse.Namespace(config_a=str(targets[0]), config_b=str(targets[1]),
                                          candidate=str(candidate), keys=["value"])
                replace = os.replace

                def fail_selected_replace(source, target):
                    source, target = Path(source), Path(target)
                    if failed_rollbacks is not None:
                        if source.suffix == ".tmp" and target == targets[1]:
                            raise OSError("injected write failure")
                        if source.suffix == ".rollback" and target.stem in failed_rollbacks:
                            raise OSError("injected rollback failure")
                    return replace(source, target)

                with patch.object(YAML_TOOL.os, "replace", fail_selected_replace), contextlib.redirect_stdout(io.StringIO()):
                    if failed_rollbacks is None:
                        self.assertEqual(YAML_TOOL.command_write_fields_pair(args), 0)
                    else:
                        with self.assertRaises((OSError, YAML_TOOL.MenuYamlError)) as caught:
                            YAML_TOOL.command_write_fields_pair(args)
                        self.assertIn("injected write failure", str(caught.exception))
                self.assertFalse(list(folder.glob("*.tmp")))
                backups = list(folder.glob("*.rollback"))
                self.assertEqual(len(backups), len(failed_rollbacks or ()))
                for target, original in zip(targets, originals):
                    if failed_rollbacks is None:
                        self.assertEqual(YAML_TOOL.load_yaml(target)[2]["value"], 3)
                    elif target.stem not in failed_rollbacks:
                        self.assertEqual(target.read_bytes(), original)
                    else:
                        backup = next(path for path in backups if path.name.startswith("." + target.name + "."))
                        self.assertEqual(backup.read_bytes(), original)
                        self.assertIn(str(backup), str(caught.exception))


if __name__ == "__main__":
    unittest.main()
