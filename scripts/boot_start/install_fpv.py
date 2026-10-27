#!/usr/bin/env python3
"""Enable FPV for next login/boot, without launching vision or interrupting current streams."""
import argparse
from pathlib import Path
import shutil
import subprocess
import sys

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT / "tools"))
from boot_start_vision import atomic_write


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mediamtx-dir", type=Path,
                        default=Path.home() / "mediamtx_v1.21.3_linux_amd64")
    media = parser.parse_args().mediamtx_dir.resolve()
    for path in (PROJECT, media):
        if any(c in str(path) for c in (' ', '\n', '\r', '"', '\\', '%')):
            raise ValueError(f"Unsupported service path: {path}")
    for name in ("ffmpeg", "xwininfo", "xprop", "xdpyinfo"):
        if not shutil.which(name):
            raise RuntimeError(f"Missing dependency: {name}")
    for name in ("mediamtx", "mediamtx.yml"):
        if not (media / name).is_file():
            raise RuntimeError(f"Missing: {media / name}")
    units = Path.home() / ".config/systemd/user"
    names = ("sp-mediamtx.service", "sp-fpv.service")
    for name in names:
        path = units / name
        if path.exists() and "Managed by sp-vision FPV installer" not in path.read_text():
            raise RuntimeError(f"Refusing to overwrite unmanaged service: {path}")
    for name in names:
        text = (Path(__file__).parent / name).read_text()
        atomic_write(units / name, text.replace("@PROJECT@", str(PROJECT)).replace("@MEDIA@", str(media)))
    subprocess.run(["systemd-analyze", "--user", "verify", *[str(units / n) for n in names]], check=True)
    subprocess.run(["systemctl", "--user", "daemon-reload"], check=True)
    subprocess.run(["systemctl", "--user", "enable", *names], check=True)
    print("Enabled MediaMTX and camera-window publisher for next boot/login; not started now.")


if __name__ == "__main__":
    main()
