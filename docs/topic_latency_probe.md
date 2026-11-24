# Sensor latency probe (`topic_latency_probe`)

`scripts/topic_latency_probe.py` subscribes to the newest back-camera image,
the 3D lidar cloud and `/front_scan` and reports how far each message's
`header.stamp` is behind the system clock.  Use it to judge whether a latency
change comes from the camera, the lidar driver, the scan conversion, or the
transport.

## Usage

```bash
# inside the GPU container (or any shell with the workspace sourced)
ros2 run usb_cam topic_latency_probe
ros2 run usb_cam topic_latency_probe --duration 30
ros2 run usb_cam topic_latency_probe --lidar-topic /vanjee/lidar_points
ros2 run usb_cam topic_latency_probe --front-scan-topic ''   # skip one topic
```

Default topics:

| key | topic |
|---|---|
| `image` | `/rgb_camera_back/image_raw` |
| `lidar` | `/vanjee/lidar` |
| `front_scan` | `/front_scan` |

The probe discovers each topic's message type through the ROS graph, so it
works with `Image`, `PointCloud2`, `LaserScan` or any message that has a
`header.stamp`.  It subscribes with sensor-data QoS (best effort), which is
compatible with both best-effort and reliable publishers.

## Reading the table

```
--- clock=system  window=20s ---
topic          rate  age ms last / avg / min / max      gap ms  vs_image ms    msgs
image         10.0Hz     95.3 /   98.1 /   90.2 /  121.4    208.0          0.0     412
lidar          9.9Hz    112.6 /  114.0 /  108.5 /  140.2    215.0         17.3     408
front_scan    10.0Hz     96.0 /   97.5 /   90.0 /  119.6    206.0          0.7     412
```

| column | meaning |
|---|---|
| `rate` | messages per second over the last `--window` seconds |
| `age` | `now - header.stamp` of the newest message: last, mean, min, max (ms) |
| `gap` | largest interval between two arrivals, since start (ms) |
| `vs_image` | `image.stamp - topic.stamp` (positive: this topic is older than the image) |
| `msgs` | messages received since start |

Expected values on the back camera with the drain-to-newest capture path:

- `image` age is roughly one frame period plus USB/decode jitter: **~100 ms**
  at 10 Hz.  A jump to 300–400 ms means the driver backlog is back (check that
  the running binary contains the drain change).
- `lidar` / `front_scan` should be in the same ballpark.  A much larger age
  points at the driver or the `pointcloud_to_laserscan` conversion.
- Large `vs_image` (hundreds of ms) means the sensors are not time-aligned even
  though each one may look healthy on its own.

## Clock caveat

`age` is only meaningful when the stamps and the node clock use the same epoch.
Device-uptime timestamps (or a zero stamp) are shown as `non-epoch`; in that
case rate/gap remain valid but `age`/`vs_image` are `n/a` and a warning is
printed at the end.  Check with:

```bash
ros2 topic echo /vanjee/lidar --field header.stamp --once
date +%s
```

If the topic runs in simulation time, start the probe with
`--ros-args -p use_sim_time:=true` so both sides use the same clock.

## Options

| option | default | meaning |
|---|---|---|
| `--duration` | 0 (until Ctrl-C) | run time in seconds |
| `--window` | 20 | statistics window in seconds |
| `--print-every` | 1 | table period in seconds |
| `--startup-timeout` | 5 | how long to wait for the topics |
| `--image-topic` / `--lidar-topic` / `--front-scan-topic` | see table | override or disable (`''`) a topic |

## Exit codes

- `0`: ran normally (including Ctrl-C);
- `1`: none of the requested topics appeared within `--startup-timeout`.
