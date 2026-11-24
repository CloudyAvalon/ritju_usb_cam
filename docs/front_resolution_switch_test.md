# Front camera resolution-switch test

`launch/front_resolution_switch_test.launch.py` repeatedly starts and stops the
**real** front `usb_cam_node_exe`, alternating between two resolutions. It uses
the same open/close lifecycle as the deployed resolution switch, so the driver
sees the same `S_FMT` / `REQBUFS` / `STREAMON` sequence, but the node itself now
reports the fault: `dropping frame` from the frame sanity checks and the
`consecutive bad frames` shutdown (which makes the node exit early).

## Run

Stop the deployed camera stack first (it owns `/dev/rgb_camera_front`), then:

```bash
ros2 launch usb_cam front_resolution_switch_test.launch.py
ros2 launch usb_cam front_resolution_switch_test.launch.py \
    interval:=6 high_res:=1280x1024 cycles:=20
```

The default parameter file is the installed `usb_cam/config/front.yaml`
(`/dev/rgb_camera_front`, `mjpeg2rgb`). Override with `params_file:=...`,
`device:=...`, `low_res:=...`, `high_res:=...`, `pixel_format:=...`.

## Output

The node now refuses `VIDIOC_REQBUFS` when the driver reports a garbage
`sizeimage` (see [sizeimage_guard.md](sizeimage_guard.md)), so a bad open fails
fast with a clear log instead of allocating `4 * sizeimage` bytes.

Each restart is one line:

```
[cycle 1] start 640x480 device=/dev/rgb_camera_front
[cycle 1] stopped after 8.0s rc=-2
[cycle 2] start 1280x1024 device=/dev/rgb_camera_front
```

- `stopped after ...` — the helper stopped the node as planned.
- `EARLY_EXIT after ...` — the node died before the interval (crash, failed
  open, or the frame-sanity escalation shutting it down on purpose). This is
  the signal to look at the node log.
- `sizeimage=...` — after every stop the helper prints the driver-reported
  `Size Image` from `v4l2-ctl --get-fmt-video`. Values above the geometry
  bound are reported as `WARNING`; once the value reaches
  `sizeimage_stop_mib` (default **100 MiB**) the run stops with exit code 2 and
  leaves the driver state for the next `usb_cam` launch.
- `summary: cycles=N early_exits=M` at the end.

## Options

| launch arg | default | meaning |
|---|---|---|
| `params_file` | installed `config/front.yaml` | ROS parameter file |
| `device` | `/dev/rgb_camera_front` | V4L2 device |
| `node_name` | `rgb_camera_front` | ROS node name per instance |
| `low_res` | `640x480` | first geometry |
| `high_res` | `1280x1024` | second geometry |
| `interval` | `8.0` | seconds each instance stays up |
| `cycles` | `0` | restarts to run, `0` = until Ctrl-C |
| `pixel_format` | empty | override the params-file fourcc |
| `io_method` | empty | override `io_method` (e.g. `userptr`) |
| `pub_raw` | `true` | publish `<node_name>/image_raw` for the test |
| `pub_compressed` | `true` | publish `<node_name>/compressed` |
| `check_sizeimage` | `true` | query `v4l2-ctl --get-fmt-video` after each stop |
| `sizeimage_stop_mib` | `100` | stop switching and exit 2 when `Size Image` reaches this many MiB |

The helper starts `usb_cam_node_exe` directly, so it applies the same topic
remappings as the deployment launch:

```
<node_name>/image_raw      <- image_raw
<node_name>/compressed     <- image_raw/compressed
<node_name>/camera_info    <- camera_info
```

The front parameter file sets `pub_raw: false` (the deployed recorder uses the
compressed topic). The test launch overrides `pub_raw`/`pub_compressed` to
`true`, so both topics exist while testing; set `pub_raw:=false` if you want to
mimic the deployed configuration exactly.

The node needs ~3 s to settle after opening, so keep `interval` above ~5 s.

## Underlying helper

`ros2 run usb_cam usb_cam_resolution_switch_test` (source:
`scripts/usb_cam_resolution_switch_test.py`) starts
`ros2 run usb_cam usb_cam_node_exe` in its own process group, waits `interval`,
sends SIGINT (then SIGTERM/SIGKILL if needed), waits for the device to be
released, and repeats. It is not installed as a normal ROS node; the launch file
is the intended entry point.
