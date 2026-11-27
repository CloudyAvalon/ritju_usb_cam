#!/usr/bin/env python3
# Copyright 2026 Evan Flynn
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Report how far the newest sensor messages lag behind the system clock.

For each topic the probe prints, every ``--print-every`` seconds:

  rate      messages per second over the last ``--window`` seconds
  age       now - header.stamp of the newest message (last / avg / min / max)
  gap       largest arrival gap seen so far
  vs_image  image.stamp - topic.stamp (positive: topic is older than image)

``age`` is only meaningful when the message stamps and the node clock share an
epoch.  Device-uptime stamps (or a zero stamp) are labelled ``non-epoch`` and
the age statistics are skipped; arrival rate/gap stay valid.

Usage:
  ros2 run usb_cam topic_latency_probe
  ros2 run usb_cam topic_latency_probe --duration 30
  ros2 run usb_cam topic_latency_probe --lidar-topic /vanjee/lidar_points
"""

import argparse
from collections import deque
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

# A stamp older/newer than this is a clock-domain mismatch, not latency.
MAX_PLAUSIBLE_DELTA_S = 3600.0

DEFAULT_TOPICS = {
    'image': '/rgb_camera_back/image_raw',
    'lidar': '/vanjee/lidar',
    'front_scan': '/front_scan',
}


def _stamp_seconds(msg):
    """Return header.stamp in seconds, or None when the message has no Header."""
    header = getattr(msg, 'header', None)
    if header is None:
        return None
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class TopicStats:
    """Rolling statistics for one subscribed topic."""

    def __init__(self, key, topic, type_name):
        self.key = key
        self.topic = topic
        self.type_name = type_name
        self.arrivals = deque()   # monotonic arrival times inside the window
        self.ages = deque()       # (monotonic, age_seconds) inside the window
        self.count = 0
        self.gap_max = 0.0
        self.last_arrival = None
        self.last_stamp = None
        self.last_age = None
        self.non_epoch = 0

    def record(self, now_wall, stamp_s, now_mono):
        self.count += 1
        if self.last_arrival is not None:
            self.gap_max = max(self.gap_max, now_mono - self.last_arrival)
        self.last_arrival = now_mono
        self.arrivals.append(now_mono)

        if stamp_s is None:
            self.non_epoch += 1
            self.last_stamp = None
            self.last_age = None
            return

        age = now_wall - stamp_s
        if stamp_s <= 0.0 or abs(age) > MAX_PLAUSIBLE_DELTA_S:
            # Device uptime, zero stamp, or a clock mismatch: not latency.
            self.non_epoch += 1
            self.last_age = None
            self.last_stamp = None
            return
        self.last_stamp = stamp_s
        self.last_age = age
        self.ages.append((now_mono, age))

    def prune(self, now_mono, window):
        while self.arrivals and now_mono - self.arrivals[0] > window:
            self.arrivals.popleft()
        while self.ages and now_mono - self.ages[0][0] > window:
            self.ages.popleft()

    def rate(self, now_mono, window):
        if len(self.arrivals) < 3:
            return None
        span = min(window, now_mono - self.arrivals[0])
        if span < 0.25:
            return None
        return len(self.arrivals) / span

    def age_summary(self):
        if not self.ages:
            return None
        values = [age for _, age in self.ages]
        return (values[-1], sum(values) / len(values), min(values), max(values))


def _discover(node, names, timeout):
    """Map every requested topic to its message type (waiting up to timeout)."""
    found = {}
    deadline = time.monotonic() + timeout
    while True:
        current = {topic: types[0] for topic, types in node.get_topic_names_and_types() if types}
        for name in names:
            if name in current and name not in found:
                found[name] = current[name]
        if len(found) == len(names) or time.monotonic() >= deadline:
            return found
        time.sleep(0.1)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Measure sensor-message age against the system clock.')
    parser.add_argument('--image-topic', default=DEFAULT_TOPICS['image'])
    parser.add_argument('--lidar-topic', default=DEFAULT_TOPICS['lidar'])
    parser.add_argument('--front-scan-topic', default=DEFAULT_TOPICS['front_scan'])
    parser.add_argument('--duration', type=float, default=0.0,
                        help='seconds to run (0 = until Ctrl-C)')
    parser.add_argument('--window', type=float, default=20.0,
                        help='statistics window in seconds (default 20)')
    parser.add_argument('--print-every', type=float, default=1.0,
                        help='table period in seconds (default 1)')
    parser.add_argument('--startup-timeout', type=float, default=5.0,
                        help='how long to wait for the topics (default 5 s)')
    args, _ = parser.parse_known_args(argv)

    wanted = {}
    for key, topic in (('image', args.image_topic),
                       ('lidar', args.lidar_topic),
                       ('front_scan', args.front_scan_topic)):
        if topic:
            wanted[key] = topic

    rclpy.init()
    node = rclpy.create_node('topic_latency_probe')
    stats = {}
    try:
        found = _discover(node, list(wanted.values()), args.startup_timeout)
        for key, topic in wanted.items():
            if topic not in found:
                print(f'warning: topic not found within {args.startup_timeout:.1f}s: {topic}',
                      file=sys.stderr)
                continue
            type_name = found[topic]
            try:
                msg_type = get_message(type_name)
            except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                print(f'warning: cannot load {type_name} for {topic}: {exc}',
                      file=sys.stderr)
                continue
            entry = TopicStats(key, topic, type_name)
            stats[topic] = entry

            def callback(msg, entry=entry):
                now = node.get_clock().now()
                now_wall = now.nanoseconds / 1e9
                entry.record(now_wall, _stamp_seconds(msg), time.monotonic())

            node.create_subscription(msg_type, topic, callback, qos_profile_sensor_data)
            print(f'subscribed {key:<10} {topic}  [{type_name}]')

        if not stats:
            print('error: no requested topic found', file=sys.stderr)
            return 1

        try:
            use_sim = bool(node.get_parameter('use_sim_time').value)
        except Exception:  # noqa: BLE001 - older rclpy may not declare it
            use_sim = False
        clock_kind = 'sim' if use_sim else 'system'
        image = next((s for s in stats.values() if s.key == 'image'), None)
        end = time.monotonic() + args.duration if args.duration > 0 else None
        next_print = time.monotonic() + args.print_every

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now_mono = time.monotonic()
            if now_mono < next_print:
                if end is not None and now_mono >= end:
                    break
                continue
            _spin_for(node, 0.05)  # let the rest of the burst update the stamps
            _print_table(stats, image, time.monotonic(), args.window, clock_kind)
            next_print += args.print_every
            if end is not None and now_mono >= end:
                break

        print('\nsummary')
        _print_table(stats, image, time.monotonic(), args.window, clock_kind, summary=True)
        for entry in stats.values():
            if entry.non_epoch and entry.non_epoch == entry.count:
                print(f'warning: {entry.topic}: all stamps are non-epoch; age cannot be '
                      'compared with this clock (arrival rate/gap are still valid)')
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _spin_for(node, seconds):
    """Process callbacks for a short while so every topic sees the same burst."""
    end = time.monotonic() + seconds
    while rclpy.ok():
        remain = end - time.monotonic()
        if remain <= 0.0:
            return
        rclpy.spin_once(node, timeout_sec=min(remain, 0.02))


def _fmt_age(entry):
    summary = entry.age_summary()
    if entry.count == 0:
        return 'waiting'
    if summary is None:
        return 'non-epoch'
    last, avg, low, high = summary
    return f'{last * 1e3:7.1f} /{avg * 1e3:7.1f} /{low * 1e3:7.1f} /{high * 1e3:7.1f}'


def _print_table(stats, image, now_mono, window, clock_kind, summary=False):
    if not summary:
        print(f'--- clock={clock_kind}  window={window:.0f}s ---')
        print(f'{"topic":<11}{"rate":>8}  {"age ms last / avg / min / max":<34}'
              f'{"gap ms":>8}  {"vs_image ms":>11}  {"msgs":>6}')
    image_stamp = image.last_stamp if image is not None else None
    for entry in stats.values():
        entry.prune(now_mono, window)
        rate = entry.rate(now_mono, window)
        rate_text = f'{rate:5.1f}Hz' if rate is not None else '   --  '
        age = _fmt_age(entry)
        gap = entry.gap_max * 1e3
        if image_stamp is not None and entry.last_stamp is not None:
            delta = image_stamp - entry.last_stamp
            if abs(delta) <= MAX_PLAUSIBLE_DELTA_S:
                vs_image = f'{delta * 1e3:11.1f}'
            else:
                vs_image = '        n/a'
        else:
            vs_image = '        n/a'
        print(f'{entry.key:<11}{rate_text}  {age:<34}{gap:8.1f}  {vs_image:>11}'
              f'  {entry.count:6d}')


if __name__ == '__main__':
    sys.exit(main())
