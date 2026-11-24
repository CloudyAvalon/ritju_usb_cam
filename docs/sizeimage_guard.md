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
   - `sizeimage == 0` is legal;
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
