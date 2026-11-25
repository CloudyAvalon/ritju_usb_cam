# Open-time sizeimage guard (OOM protection)

## Symptom

Repeatedly restarting the front/back `usb_cam` node at alternating resolutions
eventually makes the host allocate an enormous amount of memory and freeze or
reboot **before any frame is read and before the helper can print
`sizeimage=...`**.

## Root cause

In `src/usb_cam.cpp`:

1. `UsbCam::init_device()` issues `VIDIOC_S_FMT` and then goes straight to
   `init_mmap()`; it never reads `v4l2_format.fmt.pix.sizeimage` back.
2. `UsbCam::init_mmap()` calls `VIDIOC_REQBUFS` with `count = 4` unconditionally.
   The kernel `vb2` layer allocates `count * sizeimage` bytes at this point.
3. The UVC descriptor bug can report a garbage `sizeimage` (e.g. near 2^32)
   after repeated resolution switches.  With `4 * 4 GiB` requested, the host
   OOMs before `streamon`, so no node log, no frame, no sizeimage output.

`ffplay` and `v4l2-ctl --stream-mmap` do not go through this `usb_cam` open
sequence, which is why they do not trigger the same allocation.

## Fix

After `VIDIOC_S_FMT`, `init_device()` now:

1. reads the negotiated format with `VIDIOC_G_FMT`;
2. rejects implausible geometry (`0` or `> 16384` pixels per side);
3. validates `sizeimage` with the same bounds as the develop preflight:
   - `expected = width * height * device_bpp` (MJPG 3, YUYV/UYVY/YUV420 2,
     GREY 1, default 3);
   - `bound = min(expected * 4, 64 MiB)`;
   - `sizeimage == 0` is legal for the driver, but **rejected for mmap** after
     the S_FMT retries: the kernel would derive the buffer size from the
     driver's internal format, which is exactly the corrupt state that causes
     OOM;
   - `bytesperline * height` is also checked against the bound for mmap;
   - values above `0xF0000000` are uint32 garbage;
4. retries `VIDIOC_S_FMT` up to three times (the S_FMT kick heals the transient
   case);
5. if the value is still bad, throws
   `sizeimage ...; refusing VIDIOC_REQBUFS` **before** any buffer allocation.

The node therefore logs

```
[ERROR] ... Failed to configure camera: sizeimage 4294967295 looks like uint32
garbage (1280x1024 'MJPG'); refusing VIDIOC_REQBUFS
```

and exits/restarts without touching the huge allocation.  The helper's
`sizeimage=...` line then reports the value that was refused.

Unit tests: `test/test_sizeimage_guard.cpp` covers the bound, zero, garbage and
acceptance cases.

## Related

- `frame_sanity.md` — per-frame payload/SOI checks and consecutive-failure
  escalation.
- `sizeimage_fault_injection.md` — how to reproduce the driver state with
  ffplay / v4l2-ctl.

## Other allocation sites audited

The same input-validation pass also covers the remaining buffers:

| site | input | guard now |
|---|---|---|
| `image_t::set_bytes_per_line()` / `set_size_in_bytes()` | ROS `image_width` / `image_height` | overflow check + 128 MiB limit |
| `UsbCam::configure()` → `MJPEG2RGB` `av_frame_get_buffer()` | ROS dimensions | `validate_frame_geometry()` **before** the pixel-format constructor allocates AVFrames |
| `m_image_msg->data.resize()` | `get_image_size_in_bytes()` | bounded by the library limit; the old `sizeof(std::vector)` test (always true) was replaced by `.size()` |
| `init_userp()` `memalign()` ×4 | `size_in_bytes` | overflow check; size already capped |
| `init_mmap()` `mmap(buf.length)` | driver `QUERYBUF.length` | 0 or > 64 MiB → `refusing mmap` |
| `VIDIOC_REQBUFS` (mmap) | driver `sizeimage` / internal format | zero or implied `bytesperline*height` above bound → refused; logs `requesting`, `REQBUFS returned N`, `QUERYBUF[i] length=...` |
| `process_image()` / MJPEG decoder `bytes_used` | driver `bytesused` | payload bounds check is now **unconditional**; only the SOI heuristic remains behind `frame_sanity_check` |
| `undistortImage()` maps / GpuMat | message geometry | indirectly bounded by the 128 MiB image cap (maps add ~2.7× that size) |
| `get_supported_formats()` `new` | fixed-size structs | no external input |

Remaining known issue, unrelated to this fault: `init_read()` sets
`m_buffers[0].length` but never allocates `m_buffers[0].start`; the `read`
io-method is not used by the deployed configuration (mmap) and would need a
separate fix.

Limits are conservative: a 1280×1024 RGB frame is ~3.9 MiB, and the largest
deployed mode is well below 64 MiB. `kMaxImageBytes` is 128 MiB; the driver
payload cap is 64 MiB.

## OOM after "This device supports the following formats:"

That line is printed by `set_pixel_format()` immediately before it calls
`supported_formats()` → `UsbCam::get_supported_formats()`, which enumerates
`VIDIOC_ENUM_FMT` / `ENUM_FRAMESIZES` / `ENUM_FRAMEINTERVALS`.  A wedged UVC
stack can ignore the increasing index and keep returning success, so the
`push_back` loop grows the mode vector until the host OOMs, with no further
log line.

`get_supported_formats()` now:

- caps formats / sizes / intervals / total entries (32 / 64 / 32 / 256);
- stops when the same `(fourcc, width, height, numerator, denominator)` mode
  is enumerated twice (the "driver ignores the index" case);
- logs `usb_cam: enumerating supported formats` and
  `usb_cam: enumerated N capture modes` (or a cap/duplicate warning).

Two more diagnostics were added on the open path:

```
usb_cam: negotiated 1280x1024 fourcc=MJPG sizeimage=... bytesperline=...
usb_cam: requesting 4 mmap buffers via VIDIOC_REQBUFS
```

If a future OOM happens again, the last line before it now identifies the
stage: enumeration, negotiated format, or buffer request.

## Why the sizeimage guard is not sufficient (UVC `dwMaxVideoFrameSize`)

The log sequence

```
usb_cam: negotiated 1280x1024 fourcc=MJPG sizeimage=2622029 bytesperline=0
usb_cam: requesting 4 mmap buffers via VIDIOC_REQBUFS
```

proves that G_FMT reported a perfectly sane 2.6 MiB payload, yet the process
still died inside `VIDIOC_REQBUFS`.  `uvcvideo` does not size its vb2 buffers
from the V4L2 `sizeimage` field alone: `uvc_queue_setup()` uses the streaming
control's `dwMaxVideoFrameSize`, which is committed by the UVC probe/commit
path (`VIDIOC_S_PARM` on this driver) and can remain corrupted even when G_FMT
looks healthy.  That is the blind spot develop's sizeimage check also has, and
why its detection did not fully solve the problem.

Two additional diagnostics/hooks were added:

- `usb_cam: setting framerate via VIDIOC_S_PARM` / `VIDIOC_S_PARM returned`,
  then `usb_cam: after S_PARM ... sizeimage=...`;
- the format is re-read and bounded again after S_PARM, before REQBUFS;
- parameter `set_framerate` (default `true`).  Setting it to `false` skips
  `VIDIOC_S_PARM` entirely:

  ```bash
  ros2 launch usb_cam front_resolution_switch_test.launch.py set_framerate:=false
  ```

  If the OOM disappears with `set_framerate:=false`, the commit triggered by
  S_PARM is the trigger, and the driver should not be asked to change the
  frame interval in this state.  If it still OOMs, the corrupted
  `dwMaxVideoFrameSize` is independent of S_PARM and only USERPTR/READ or a
  uvcvideo/kernel fix avoids it.

`ffplay`/FFmpeg use a different V4L2 open/read path and do not perform this
exact `usb_cam` sequence, which is why they can still show an image.

## Frame-rate guard before VIDIOC_S_PARM

`init_device()` now reads the current interval with `VIDIOC_G_PARM` before
committing:

```
usb_cam: current framerate 15.000000 fps, requested 15
usb_cam: framerate already matches within 0.1 fps; skipping VIDIOC_S_PARM
```

- `|current_fps - requested_fps| < 0.1` → skip S_PARM (no UVC probe/commit);
- the device does not advertise `V4L2_CAP_TIMEPERFRAME` → skip S_PARM;
- otherwise set `timeperframe = 1/requested` and commit as before;
- `set_framerate:=false` still skips the whole block.

Node restarts therefore do not repeatedly commit the frame interval when the
device already runs at the requested rate, which is the state that triggers
the corrupted `dwMaxVideoFrameSize` in the first place. The comparison uses
milli-fps rounding so the exact 0.1 boundary is not affected by floating point
(15.1 vs 15 counts as different).

The helper prints the device's `--get-parm` value after every stop, so this
guard can be verified together with `check_framerate`.
