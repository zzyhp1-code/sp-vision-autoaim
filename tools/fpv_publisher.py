#!/usr/bin/env python3
"""Publish camera frames; no X11, camera SDK or control outputs."""
import argparse
import fcntl
import logging
import math
import os
from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import threading
import time
from boot_start_vision import clean_environment, stop_child

URL = 'rtsp://127.0.0.1:8554/fpv'
LOG = logging.getLogger('fpv')
HEADER = struct.Struct('<4sQI')
DEFAULT_VIEW_GAMMA = 3.2
# Apply the viewing lift in full-range RGB, before conversion to video-range YUV.
# eq=gamma on limited-range YUV also lifts code 16 (nominal black) into gray.
def viewing_gamma(value):
    gamma = float(value)
    if not math.isfinite(gamma) or not 1.0 <= gamma <= 5.0:
        raise ValueError('viewing gamma must be finite and between 1.0 and 5.0')
    return gamma


def view_filter(gamma=DEFAULT_VIEW_GAMMA):
    exponent = 1.0 / viewing_gamma(gamma)
    expression = f"'pow(val/255,{exponent:.12g})*255'"
    return ('scale=in_range=pc:out_range=pc,format=rgb24,'
            f'lutrgb=r={expression}:g={expression}:b={expression},'
            'scale=in_range=pc:out_range=tv:out_color_matrix=bt709,format=yuv420p')


def socket_path():
    runtime = Path(f'/run/user/{os.getuid()}')
    base = runtime if runtime.is_dir() else Path('/tmp')
    return Path(os.environ.get('SP_VISION_FPV_SOCKET',
                              str(base / f'sp-vision-fpv-{os.getuid()}/camera.sock')))


def command(gamma=DEFAULT_VIEW_GAMMA, input_codec='bmp'):
    if input_codec not in ('bmp', 'mjpeg'):
        raise ValueError('unsupported preview codec')
    # RGB lift preserves black/white; it is not equivalent to YUV eq gamma.
    return ['/usr/bin/ffmpeg', '-nostdin', '-nostats', '-loglevel', 'warning',
            '-fflags', 'nobuffer', '-flags', 'low_delay', '-analyzeduration', '0',
            '-probesize', '32', '-f', 'image2pipe', '-framerate', '30',
            '-use_wallclock_as_timestamps', '1',
            '-vcodec', input_codec, '-i', 'pipe:0', '-vsync', '0', '-vf', view_filter(gamma),
            '-c:v', 'libx264', '-threads', '2', '-preset', 'ultrafast',
            '-tune', 'zerolatency', '-profile:v', 'baseline', '-pix_fmt', 'yuv420p',
            '-color_range', 'tv', '-colorspace', 'bt709',
            '-color_primaries', 'bt709', '-color_trc', 'bt709',
            '-crf', '20', '-maxrate', '8M', '-bufsize', '4M',
            '-g', '15', '-bf', '0', '-an', '-flush_packets', '1', '-muxdelay', '0', '-f', 'rtsp',
            '-rtsp_transport', 'tcp', URL]


def existing_publisher():
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            args = (proc / 'cmdline').read_bytes().split(b'\0')
            if args and Path(os.fsdecode(args[0])).name == 'ffmpeg' and URL.encode() in args:
                return int(proc.name)
        except OSError:
            pass
    return None


def ready():
    try:
        with socket.create_connection(('127.0.0.1', 8554), timeout=1):
            return True
    except OSError:
        return False


def receive_exact(connection, size):
    result = bytearray()
    while len(result) < size:
        data = connection.recv(size - len(result))
        if not data:
            raise EOFError('camera preview disconnected')
        result.extend(data)
    return bytes(result)


def receive_frame(connection, metadata=None):
    magic, timestamp, size = HEADER.unpack(receive_exact(connection, HEADER.size))
    if magic not in (b'SPF1', b'SPF2') or not 4 <= size <= 12 * 1024 * 1024:
        raise ValueError('invalid camera preview header')
    frame = receive_exact(connection, size)
    age = (time.monotonic_ns() - timestamp) / 1e9
    if not 0 <= age <= 0.5:
        return None
    if metadata is not None:
        metadata['capture_ns'] = timestamp
        metadata['receive_age_ms'] = age * 1000
    if magic == b'SPF1':
        if not frame.startswith(b'\xff\xd8') or not frame.endswith(b'\xff\xd9'):
            raise ValueError('invalid legacy JPEG preview')
    else:
        if len(frame) < 54 or frame[:2] != b'BM':
            raise ValueError('invalid BMP preview')
        total, offset = struct.unpack_from('<I', frame, 2)[0], struct.unpack_from('<I', frame, 10)[0]
        dib, width, height, planes, bits, compression = struct.unpack_from('<IiiHHI', frame, 14)
        stride = (width * 3 + 3) // 4 * 4
        if (dib != 40 or not 0 < width <= 2048 or not 0 < abs(height) <= 1536 or
                planes != 1 or bits != 24 or compression != 0 or offset != 54 or
                total != len(frame) or offset + stride * abs(height) != total):
            raise ValueError('unsupported BMP layout')
    return frame


def write_frame(pipe, data):
    deadline = time.monotonic() + 0.25
    view = memoryview(data)
    while view:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([], [pipe], [], remaining)[1]:
            raise TimeoutError('encoder backpressure; discard stream instead of queueing')
        try:
            count = os.write(pipe.fileno(), view)
        except BlockingIOError:
            continue
        if count <= 0:
            raise BrokenPipeError('encoder closed')
        view = view[count:]


def publish(connection, stop, gamma=DEFAULT_VIEW_GAMMA):
    child = None
    input_codec = None
    last_fresh = time.monotonic()
    report_start = last_fresh
    ages, writes = [], []
    connection.settimeout(1)
    try:
        while not stop.is_set():
            metadata = {}
            frame = receive_frame(connection, metadata)
            if frame is None:
                if time.monotonic() - last_fresh > 1:
                    raise TimeoutError('camera frames remain stale')
                continue
            last_fresh = time.monotonic()
            codec = 'bmp' if frame.startswith(b'BM') else 'mjpeg'
            if child is None:
                input_codec = codec
                child = subprocess.Popen(command(gamma, input_codec), env=clean_environment(), stdin=subprocess.PIPE,
                                         bufsize=0, start_new_session=True)
                os.set_blocking(child.stdin.fileno(), False)
                LOG.info('publishing camera frames, encoder PID %s', child.pid)
            elif codec != input_codec:
                raise ValueError('preview format changed; reconnect encoder')
            if child.poll() is not None:
                raise BrokenPipeError('encoder exited')
            write_start = time.monotonic()
            write_frame(child.stdin, frame)
            writes.append((time.monotonic() - write_start) * 1000)
            ages.append(metadata['receive_age_ms'])
            elapsed = time.monotonic() - report_start
            if elapsed >= 5:
                LOG.info('[FPVLATENCY] input_fps=%.2f receive_age_p90_ms=%.1f write_p90_ms=%.1f '
                         '(not browser end-to-end latency)', len(ages)/elapsed,
                         sorted(ages)[int((len(ages)-1)*.9)], sorted(writes)[int((len(writes)-1)*.9)])
                ages.clear(); writes.clear(); report_start = time.monotonic()
    finally:
        if child is not None:
            child.stdin.close()
            stop_child(child)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true', help='inspect without opening camera')
    parser.add_argument('--gamma', type=viewing_gamma, default=DEFAULT_VIEW_GAMMA,
                        help='viewing brightness only, 1.0 to 5.0 (default: 3.2)')
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
    path = socket_path()
    if args.check:
        print(f'MediaMTX ready={ready()}; preview socket={path}; exists={path.exists()}; '
              f'existing publisher={existing_publisher()}; viewing gamma={args.gamma}; '
              'frames arrive only during capture')
        return
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    if path.parent.stat().st_uid != os.getuid() or path.parent.stat().st_mode & 0o077:
        raise RuntimeError('preview socket directory must be private and owned by this user')
    with (path.parent / 'publisher.lock').open('a') as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            LOG.info('another supervisor already running')
            return
        path.unlink(missing_ok=True)
        stop = threading.Event()
        for sig in (signal.SIGTERM, signal.SIGINT):
            signal.signal(sig, lambda *_: stop.set())
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
            server.bind(str(path))
            server.listen(1)
            server.settimeout(1)
            LOG.info('waiting for camera frames on %s (no desktop needed)', path)
            try:
                while not stop.is_set():
                    if existing_publisher() or not ready():
                        stop.wait(2)
                        continue
                    try:
                        connection, _ = server.accept()
                    except socket.timeout:
                        continue  # idle without a camera: no repeating log/encoder work
                    try:
                        with connection:
                            publish(connection, stop, args.gamma)
                    except (OSError, EOFError, ValueError) as exc:
                        if not stop.is_set():
                            LOG.info('preview waiting/reconnecting: %s', exc)
                        stop.wait(0.2)
            finally:
                path.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
