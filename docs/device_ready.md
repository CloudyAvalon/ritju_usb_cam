# Device open delay

`UsbCam::open_device()` is a **single attempt**: `stat()` + `open()`.  If the
device node is missing or cannot be opened, the node fails and the launcher /
helper restarts the whole process (with backoff).  The driver does not retry
the open itself.

After a **successful open**, and before the first device-state ioctl
(`VIDIOC_QUERYCAP`, `VIDIOC_G_FMT`, `VIDIOC_S_FMT`, ...), the library waits
once for a configurable delay.  This covers a device that has just been reset /
re-enumerated: the fd is open, but the first `GET_*` would otherwise race with
the device becoming ready.

## Parameter

```yaml
/**:
  ros__parameters:
    device_open_delay_ms: 2000   # default 2000 ms
```

- `0` disables the wait;
- read from the same camera YAML as `framerate`;
- absent → default `2000` ms.

The wait is a single `sleep_for`, **not** a retry loop.  If the device is still
not ready after that, the first state ioctl fails normally, the node logs
`Failed to configure camera: ...`, and exits so the launcher/helper restarts
it.

`VIDIOC_S_PARM` keeps its own single-attempt logic (skip when the current
framerate already matches within 0.1 fps), and per-frame `G_FMT` in
`read_frame()` is unchanged.

`UsbCamNode::init()` no longer scans `available_devices()` before opening —
that scan is itself a `VIDIOC_QUERYCAP` and could fail while the device is
still resetting.  The node opens the device first, waits
`device_open_delay_ms`, and only then reads state.  If the open fails, the
normal `Failed to configure camera: ...` path restarts the node.

## Log

```text
usb_cam: device opened; waiting 2000 ms before reading state
```

This is a readiness wait, not an OOM fix: it does not change
`dwMaxVideoFrameSize`.  Use `set_framerate:=false`, `publish_rate`, or
`io_method:=userptr` for the sizeimage/OOM issue.
