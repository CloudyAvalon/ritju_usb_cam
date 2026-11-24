#!/usr/bin/env python3
"""Restart the usb_cam node at two resolutions, repeatedly.

The node is started with ``ros2 run usb_cam usb_cam_node_exe`` and stopped with
SIGINT every ``--interval`` seconds: the same lifecycle that
``rgb_camera_manager`` uses when it switches resolutions. Each restart re-opens
the V4L2 device, which is what triggers the sizeimage corruption under test.

Watch the node log for ``dropping frame`` / ``consecutive bad frames``. A node
that exits before the interval is reported as ``EARLY_EXIT``.
"""

import argparse
import os
import re
import shutil
import signal
import subprocess
import sys
import time

RESOLUTION_RE = re.compile(r'^(\d+)x(\d+)$')

# Geometry bound used for the informational warning (develop's preflight):
#   expected = width * height * device_bpp
#   bound    = min(expected * 4, 64 MiB), sizeimage == 0 is legal.
SIZEIMAGE_FACTOR = 4
SIZEIMAGE_ABS_CAP = 64 * 1024 * 1024
# Stop the switching loop once the driver reports a clearly bogus sizeimage.
# 100 MiB is far above any healthy frame (a 1280x1024 MJPG buffer is ~4 MiB).
SIZEIMAGE_STOP_MIB = 100
SIZEIMAGE_STOP_BYTES = SIZEIMAGE_STOP_MIB * 1024 * 1024
SIZEIMAGE_BPP = {
    'MJPG': 3, 'JPEG': 3,
    'YUYV': 2, 'UYVY': 2, 'YU12': 2, 'YV12': 2,
    'GREY': 1, 'Y800': 1,
}


def parse_resolution(value):
    match = RESOLUTION_RE.match(value)
    if not match:
        raise argparse.ArgumentTypeError(f'expected WxH, got {value!r}')
    return int(match.group(1)), int(match.group(2))


def node_command(args, width, height):
    command = [
        'ros2', 'run', 'usb_cam', 'usb_cam_node_exe', '--ros-args',
        '--params-file', args.params_file,
        '-r', f'__node:={args.node_name}',
        '-p', f'video_device:={args.device}',
        '-p', f'image_width:={width}',
        '-p', f'image_height:={height}',
    ]
    if args.pixel_format:
        command += ['-p', f'pixel_format:={args.pixel_format}']
    return command


def str2bool(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def read_sizeimage(device, v4l2ctl):
    """Return (info, error) from `v4l2-ctl --get-fmt-video`."""
    try:
        result = subprocess.run(
            [v4l2ctl, '-d', device, '--get-fmt-video'],
            capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, str(exc)
    if result.returncode != 0:
        return None, result.stderr.strip() or f'exit {result.returncode}'
    text = result.stdout
    wh = re.search(r'Width/Height\s*:\s*(\d+)/(\d+)', text)
    size = re.search(r'Size Image\s*:\s*(\d+)', text)
    fourcc = re.search(r"Pixel Format\s*:\s*'([^']+)'", text)
    if not (wh and size):
        return None, 'Width/Height or Size Image missing'
    return {
        'width': int(wh.group(1)),
        'height': int(wh.group(2)),
        'size': int(size.group(1)),
        'fourcc': fourcc.group(1) if fourcc else '',
    }, ''


def sizeimage_anomaly(info, stop_bytes=SIZEIMAGE_STOP_BYTES):
    """Return (reason, should_stop) for the driver-reported sizeimage.

    ``should_stop`` is true once the value reaches ``stop_bytes`` (default
    100 MiB).  Values that merely exceed the geometry bound are reported as a
    warning without ending the run, so the loop can reach the large garbage
    state first.
    """
    size = info['size']
    if size >= stop_bytes:
        return ('sizeimage %d >= %d bytes (%.1f MiB) (%dx%d %s)' %
                (size, stop_bytes, stop_bytes / (1024.0 * 1024.0),
                 info['width'], info['height'], info['fourcc'])), True
    bpp = SIZEIMAGE_BPP.get(info['fourcc'].upper(), 3)
    expected = info['width'] * info['height'] * bpp
    bound = min(expected * SIZEIMAGE_FACTOR, SIZEIMAGE_ABS_CAP)
    if size > bound:
        return ('sizeimage %d above geometry bound %d (%dx%d %s)' %
                (size, bound, info['width'], info['height'], info['fourcc'])), False
    return '', False


def stop_node(process):
    if process.poll() is not None:
        return process.returncode
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return process.poll()
    try:
        return process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return process.wait(timeout=1.0)
    try:
        return process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        return process.wait(timeout=5.0)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Restart the usb_cam node at two resolutions in a loop.')
    parser.add_argument('--params-file', required=True,
                        help='ROS parameter file for the camera')
    parser.add_argument('--device', default='/dev/rgb_camera_front',
                        help='V4L2 device path (default /dev/rgb_camera_front)')
    parser.add_argument('--node-name', default='rgb_camera_front',
                        help='ROS node name (default rgb_camera_front)')
    parser.add_argument('--low', type=parse_resolution, default=(640, 480),
                        help='first resolution, e.g. 640x480')
    parser.add_argument('--high', type=parse_resolution, default=(1280, 1024),
                        help='second resolution, e.g. 1280x1024')
    parser.add_argument('--interval', type=float, default=8.0,
                        help='seconds each node instance stays up (default 8)')
    parser.add_argument('--cycles', type=int, default=0,
                        help='number of restarts, 0 = until Ctrl-C')
    parser.add_argument('--settle', type=float, default=1.5,
                        help='seconds between instances (default 1.5)')
    parser.add_argument('--pixel-format', default='',
                        help='override pixel_format (default: from params file)')
    parser.add_argument('--check-sizeimage', type=str2bool, default=True,
                        metavar='BOOL',
                        help='query v4l2-ctl after each stop (default true)')
    parser.add_argument('--v4l2ctl', default='v4l2-ctl',
                        help='v4l2-ctl binary (default v4l2-ctl)')
    parser.add_argument('--sizeimage-stop-mib', type=float,
                        default=SIZEIMAGE_STOP_MIB,
                        help='stop when Size Image >= this value in MiB '
                             '(default 100)')
    args = parser.parse_args(argv)

    if args.interval <= 3.0:
        print('warning: interval <= 3 s overlaps the node startup settle time',
              file=sys.stderr)

    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    if args.check_sizeimage and shutil.which(args.v4l2ctl) is None:
        print(f'warning: {args.v4l2ctl} not found; sizeimage checks disabled',
              file=sys.stderr)
        args.check_sizeimage = False

    resolutions = [args.low, args.high]
    cycle = 0
    early_exits = 0
    while not stop_requested and (args.cycles == 0 or cycle < args.cycles):
        width, height = resolutions[cycle % 2]
        cycle += 1
        print(f'[cycle {cycle}] start {width}x{height} device={args.device}',
              flush=True)
        started = time.monotonic()
        process = subprocess.Popen(
            node_command(args, width, height), start_new_session=True)
        try:
            while (not stop_requested and
                   time.monotonic() - started < args.interval):
                if process.poll() is not None:
                    break
                time.sleep(0.1)
            returncode = stop_node(process)
        except Exception:
            stop_node(process)
            raise
        elapsed = time.monotonic() - started
        if elapsed < args.interval - 0.5:
            early_exits += 1
            print(f'[cycle {cycle}] EARLY_EXIT after {elapsed:.1f}s '
                  f'rc={returncode}', flush=True)
        else:
            print(f'[cycle {cycle}] stopped after {elapsed:.1f}s rc={returncode}',
                  flush=True)
        if args.check_sizeimage and not stop_requested:
            info, error = read_sizeimage(args.device, args.v4l2ctl)
            if info is None:
                print(f'[cycle {cycle}] sizeimage: {error}', flush=True)
            else:
                print(f"[cycle {cycle}] sizeimage={info['size']} "
                      f"({info['width']}x{info['height']} '{info['fourcc']}')",
                      flush=True)
                stop_bytes = args.sizeimage_stop_mib * 1024 * 1024
                reason, should_stop = sizeimage_anomaly(info, stop_bytes)
                if reason:
                    level = 'ABNORMAL' if should_stop else 'WARNING'
                    print(f'[cycle {cycle}] {level} SIZEIMAGE: {reason}',
                          flush=True)
                if should_stop:
                    print('capture stopped; the driver state is left for the '
                          'next usb_cam launch', flush=True)
                    print(f'summary: cycles={cycle} early_exits={early_exits}',
                          flush=True)
                    return 2
        if stop_requested:
            break
        time.sleep(args.settle)

    print(f'summary: cycles={cycle} early_exits={early_exits}', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
