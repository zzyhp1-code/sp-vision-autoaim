#!/usr/bin/env python3
"""Publish only the standard camera window; never starts vision or opens the camera."""
import argparse
import fcntl
import logging
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import threading

from boot_start_vision import clean_environment, display_environment, stop_child

URL = "rtsp://127.0.0.1:8554/fpv"
LOG = logging.getLogger("fpv")


def window_candidates(tree):
    return re.findall(r'^\s*(0x[0-9a-fA-F]+) "reprojection": \("standard" "Standard"\)',
                      tree, re.MULTILINE)


def camera_window(env, binary):
    tree = subprocess.run(["xwininfo", "-root", "-tree"], env=env,
                          capture_output=True, text=True, timeout=3)
    for window in window_candidates(tree.stdout):
        prop = subprocess.run(["xprop", "-id", window, "_NET_WM_PID"], env=env,
                              capture_output=True, text=True, timeout=3)
        match = re.search(r"=\s*(\d+)", prop.stdout)
        if not match:
            continue
        try:
            # readlink also works for a running executable replaced by a new build.
            executable = os.readlink(f"/proc/{match[1]}/exe").removesuffix(" (deleted)")
            if executable == str(binary):
                info = subprocess.run(["xwininfo", "-id", window], env=env,
                                      capture_output=True, text=True, timeout=3)
                if "Map State: IsViewable" in info.stdout:
                    return window
        except OSError:
            pass
    return None


def existing_publisher():
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        try:
            args = (proc / "cmdline").read_bytes().split(b"\0")
            if args and Path(os.fsdecode(args[0])).name == "ffmpeg" and URL.encode() in args:
                return int(proc.name)
        except OSError:
            pass
    return None


def command(window):
    # Additional viewing-only lift after the display's existing Gamma correction.
    return ["/usr/bin/ffmpeg", "-nostdin", "-nostats", "-loglevel", "warning",
            "-f", "x11grab", "-framerate", "30", "-window_id", window,
            "-draw_mouse", "0", "-i", ":0.0", "-vf",
            "scale=trunc(iw/2)*2:trunc(ih/2)*2,eq=gamma=1.25", "-c:v", "libx264",
            "-preset", "ultrafast", "-tune", "zerolatency", "-profile:v", "baseline",
            "-pix_fmt", "yuv420p", "-b:v", "3M", "-g", "30", "-bf", "0",
            "-an", "-f", "rtsp", "-rtsp_transport", "tcp", URL]


def ready(project):
    env = display_environment(clean_environment())
    if env is None:
        return None, None, "waiting for physical X11 desktop"
    window = camera_window(env, project / "build/standard")
    if window is None:
        return env, None, "waiting for standard reprojection window"
    try:
        with socket.create_connection(("127.0.0.1", 8554), timeout=1):
            pass
    except OSError:
        return env, None, "waiting for MediaMTX port 8554"
    return env, window, "ready"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="inspect readiness without publishing")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    project = Path(__file__).resolve().parents[1]
    if args.check:
        env, window, reason = ready(project)
        print(reason, "window=", window, "existing publisher=", existing_publisher())
        return
    lock = project / "logs/xuc-bringup/fpv_publisher.lock"
    lock.parent.mkdir(parents=True, exist_ok=True)
    with lock.open("a") as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            LOG.info("another supervisor already running")
            return
        stop = threading.Event()
        for sig in (signal.SIGTERM, signal.SIGINT):
            signal.signal(sig, lambda *_: stop.set())
        child = None
        last = None
        try:
            while not stop.is_set():
                try:
                    env, window, reason = ready(project)
                    other = existing_publisher()
                    if other:
                        window, reason = None, f"waiting for existing publisher PID {other}"
                    if reason != last:
                        LOG.info(reason)
                        last = reason
                    if window:
                        cmd = command(window)
                        cmd[cmd.index("-i") + 1] = env["DISPLAY"] + ".0"
                        child = subprocess.Popen(cmd, env=env, stdin=subprocess.DEVNULL,
                                                 start_new_session=True)
                        LOG.info("publishing window %s, PID %s", window, child.pid)
                        while not stop.wait(1) and child.poll() is None:
                            if camera_window(env, project / "build/standard") != window:
                                LOG.info("window closed/changed; rebinding")
                                break
                        stop_child(child)
                        child = None
                        last = None
                except (OSError, subprocess.TimeoutExpired) as exc:
                    LOG.warning("capture check failed: %s", exc)
                    if child is not None:
                        stop_child(child)
                        child = None
                stop.wait(2)
        finally:
            if child is not None:
                stop_child(child)


if __name__ == "__main__":
    main()
