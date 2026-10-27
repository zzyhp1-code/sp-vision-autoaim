#!/usr/bin/env python3
"""Boot supervisor for the existing standard binary. No serial commands or algorithm changes."""

import argparse
import fcntl
import logging
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import threading
import time

LOG = logging.getLogger("vision-boot")
ROS_SETUP = Path("/opt/ros/humble/setup.bash")
RUNTIME_SETUP = r'''
set -e
source /opt/ros/humble/setup.bash
if [[ -r /opt/MVS/setup.sh ]]; then source /opt/MVS/setup.sh; fi
export MVCAM_SDK_PATH=/opt/MVS
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol
export ALLUSERSPROFILE=/opt/MVS/MVFG
export LD_LIBRARY_PATH="/opt/MVS/lib/64:/opt/MVS/lib/32:${LD_LIBRARY_PATH:-}"
exec "$@"
'''


def clean_environment(source=None):
    env = dict(os.environ if source is None else source)
    for key in ("LD_PRELOAD", "LD_LIBRARY_PATH", "PYTHONPATH", "BASH_ENV", "ENV"):
        env.pop(key, None)
    env["PATH"] = "/usr/local/bin:/usr/bin:/bin"
    return env


def atomic_write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=path.name + ".", suffix=".tmp", delete=False) as out:
            temporary = Path(out.name)
            out.write(text)
            out.flush()
            os.fsync(out.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def configuration(project, home=None):
    import yaml
    source = (project / "configs/standard.yaml").read_text(encoding="utf-8")
    config = yaml.safe_load(source)
    home = Path.home() if home is None else home
    state_dir = Path(os.environ.get("XDG_CONFIG_HOME", str(home / ".config"))) / "sp-vision"
    saved = state_dir / "field_autoaim_enemy_color"
    color = saved.read_text().strip() if saved.is_file() else ""
    if color not in ("red", "blue"):
        color = config.get("enemy_color")
    if color not in ("red", "blue"):
        raise ValueError("无法确定已有锁定颜色")
    pattern = r"^[ \t]*enemy_color[ \t]*:.*$"
    if len(re.findall(pattern, source, re.MULTILINE)) != 1:
        raise ValueError("正式配置必须只有一个 enemy_color 字段")
    runtime = re.sub(pattern, f'enemy_color: "{color}"', source, count=1, flags=re.MULTILINE)
    return config, color, runtime


def runtime_command(*args):
    return ["/bin/bash", "-c", RUNTIME_SETUP, "vision-boot", *map(str, args)]


def static_check(project):
    binary = project / "build/standard"
    if not binary.is_file() or not os.access(binary, os.X_OK):
        raise ValueError(f"主程序不可执行：{binary}")
    if not ROS_SETUP.is_file():
        raise ValueError(f"ROS 环境缺失：{ROS_SETUP}")
    for name in ("tiny_resnet.onnx", "yolo26.xml", "yolo26.bin"):
        path = project / "assets" / name
        if not path.is_file() or not path.stat().st_size:
            raise ValueError(f"已有运行模型缺失：{path}")
    result = subprocess.run(runtime_command("/usr/bin/ldd", binary), env=clean_environment(),
                            text=True, capture_output=True, timeout=20)
    if result.returncode or "not found" in result.stdout:
        raise ValueError(f"运行库检查失败：{result.stdout}\n{result.stderr}")


def display_environment(env):
    # Prefer the physical GDM desktop, not a remote desktop imported into the user manager.
    auths = [Path(f"/run/user/{os.getuid()}/gdm/Xauthority"), Path.home() / ".Xauthority"]
    if env.get("XAUTHORITY"):
        auths.append(Path(env["XAUTHORITY"]))
    for display in (":0", ":1"):
        if not Path("/tmp/.X11-unix/X" + display[1:]).exists():
            continue
        for auth in dict.fromkeys(auths):
            if not auth.is_file():
                continue
            candidate = dict(env, DISPLAY=display, XAUTHORITY=str(auth))
            try:
                probe = subprocess.run(["/usr/bin/xdpyinfo"], env=candidate,
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2)
            except subprocess.TimeoutExpired:
                continue
            if probe.returncode == 0:
                return candidate
    return None


def readiness(config):
    env = clean_environment()
    serial = Path(config["xuc_serial_port"])
    if not serial.exists() or not os.access(serial, os.R_OK | os.W_OK):
        return None, f"等待串口及访问权限：{serial}"
    vid, pid = config.get("vid_pid", "2bdf:0001").lower().split(":")
    found = False
    for device in Path("/sys/bus/usb/devices").glob("*"):
        try:
            if (device / "idVendor").read_text().strip() == vid and (device / "idProduct").read_text().strip() == pid:
                found = True
                break
        except OSError:
            continue
    if not found:
        return None, "等待现有 USB 相机"
    if not config.get("competition_headless", False):
        env = display_environment(env)
        if env is None:
            return None, "等待开机自动登录的 X11 桌面"
    return env, "桌面、相机和串口已就绪（未打开设备）"


def existing_standard(binary):
    for proc in Path("/proc").glob("[0-9]*"):
        try:
            if (proc / "exe").resolve(strict=True) == binary.resolve():
                return int(proc.name)
        except OSError:
            continue
    return None


def stop_child(child):
    for sig, timeout in ((signal.SIGINT, 10), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        try:
            os.killpg(child.pid, sig)
        except ProcessLookupError:
            break
        try:
            child.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue


def supervise(project, color, runtime_text, env, stop):
    runtime = project / "logs/xuc-bringup"
    config_path = runtime / "field_autoaim.runtime.yaml"
    pid_path = runtime / "field_autoaim.pid"
    last_log = runtime / "field_autoaim.last_log"
    atomic_write(config_path, runtime_text)
    log_path = runtime / time.strftime("%Y-%m-%d") / f"one_click_autoaim_{color}_{time.strftime('%H%M%S')}.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    atomic_write(last_log, str(log_path) + "\n")
    child = None
    try:
        with log_path.open("ab", buffering=0) as output:
            child = subprocess.Popen(runtime_command("/usr/bin/stdbuf", "-oL", "-eL",
                                     project / "build/standard", config_path), cwd=project,
                                     env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            atomic_write(pid_path, str(child.pid) + "\n")
            LOG.info("启动现有 standard，PID=%s，颜色=%s，日志=%s", child.pid, color, log_path)
            while child.poll() is None and not stop.wait(0.2):
                pass
            if stop.is_set():
                stop_child(child)
            result = child.wait()
            LOG.info("standard 退出，状态=%s", result)
            # Operator stop is intentional; do not immediately resurrect the program.
            return 0 if stop.is_set() or result in (0, -2, -15, 130, 143) else 1
    finally:
        if child is not None and child.poll() is None:
            stop_child(child)
        if child is not None and pid_path.is_file() and pid_path.read_text().strip() == str(child.pid):
            pid_path.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--check", action="store_true", help="仅检查依赖和就绪状态，不启动主程序或打开设备")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [BOOT] %(message)s")
    project = args.project.resolve()
    config, color, runtime_text = configuration(project)
    static_check(project)
    if args.check:
        env, message = readiness(config)
        print(f"配置与运行库检查通过；颜色={color}；{message}")
        if env is not None:
            print(f"DISPLAY={env.get('DISPLAY', '(headless)')}")
        return 0 if env is not None else 2
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    lock_path = project / "logs/xuc-bringup/vision_boot.lock"
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("a") as lease:
        try:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            LOG.info("开机入口已运行，不创建重复实例")
            return 0
        last_reason = None
        while not stop.is_set():
            pid = existing_standard(project / "build/standard")
            if pid is not None:
                LOG.info("已有 standard PID=%s，不重复启动", pid)
                return 0
            env, reason = readiness(config)
            if reason != last_reason:
                LOG.info(reason)
                last_reason = reason
            if env is not None:
                if not stop.is_set():
                    return supervise(project, color, runtime_text, env, stop)
                break
            stop.wait(1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
