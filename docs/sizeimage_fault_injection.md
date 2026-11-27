# Reproducing the UVC sizeimage fault on real hardware

Repeated resolution switches can leave some UVC devices reporting a garbage
`Size Image` (near 2^32, or far above what the geometry implies). `usb_cam`'s
`develop` branch healed that at open time; `dev_perf` intentionally does not,
so the fault can be used to exercise the driver frame checks and the
consecutive-failure shutdown.

Two scripts are available:

- `scripts/ffplay_oom_probe.sh` — the **faithful develop reproducer**: it
  alternates *low resolution + front/left/right sidecars* with *high
  resolution alone*, using windowed ffplay with the format left on auto. This
  is the pattern that reproduced the sizeimage corruption in develop.
- `scripts/ffplay_switch_diag.sh` — the diagnostic matrix (hold / reopen /
  switch / setfmt / stream). Its ffplay modes now default to the same
  known-good form as the OOM probe; `setfmt`/`stream` are explicit-format,
  headless variants.

Both embed the sizeimage check (single-file, nothing to copy alongside) and are
host-side diagnostics; they are deliberately not installed into the ROS package.

## Prerequisites

- host access to `/dev/rgb_camera_back` (the node is a host udev symlink);
- `ffplay` and `v4l2-ctl` (`v4l-utils`) on the host;
- the camera stack **stopped** — the running `rgb_camera_back` composable node
  owns the V4L2 fd, so stop the GPU container (or switch the camera to `idle`)
  before starting the diagnostic;
- headless host: do **not** use ffplay. `ffplay -nodisp` (ffplay 4.4) disables
  the display, but for a video-only input it exits with
  `Failed to open file ... or configure filtergraph` and never runs a capture
  loop. Without `-nodisp`, a headless host fails at SDL init
  (`Could not initialize SDL - No available video device`) before opening the
  device. Use the `setfmt` or `stream` mode instead; they need no display.

## Explicit format switching (optional)

A fresh UVC device usually reports **YUYV** until something sets MJPG.
`ffplay -video_size WxH` only requests the geometry; the fourcc negotiation is
left to ffmpeg. The develop reproducer deliberately kept this behaviour (format
auto) and still hit the sizeimage bug, so do **not** assume MJPG has to be
forced.

If an explicit format is needed, three options exist:

```bash
# 1) opt-in pre-set before each ffplay open (may hide the bug)
SETFMT_BEFORE_OPEN=1 MODES=switch ... ./scripts/ffplay_switch_diag.sh

# 2) pure format switching, no ffplay (headless)
MODES=setfmt PER_INTERVAL_RUNS=50 PIXEL_FORMAT=MJPG ...
  ./scripts/ffplay_switch_diag.sh

# 3) format switch + real capture, no ffplay (headless)
MODES=stream PER_INTERVAL_RUNS=50 PIXEL_FORMAT=MJPG STREAM_COUNT=30 ...
  ./scripts/ffplay_switch_diag.sh
```

`setfmt` uses

```bash
v4l2-ctl -d /dev/rgb_camera_back \
  --set-fmt-video=width=1280,height=1024,pixelformat=MJPG
```

and `stream` additionally drains `STREAM_COUNT` frames with
`v4l2-ctl --stream-mmap --stream-count=N --stream-to=/dev/null`
(REQBUFS/STREAMON/STREAMOFF, headless). These modes are useful for driving the
driver into the bad state, but the multi-camera phase pattern of
`ffplay_oom_probe.sh` is the one that reproduced it reliably.

### Why the old script did not reproduce it

| aspect | develop reproducer | old `ffplay_switch_diag.sh` |
|---|---|---|
| camera phases | low+sidecars <-> **high-solo** (sidecars stopped) | sidecars always on (or off); only back switched |
| sidecars | `front left right`, on by default | empty by default |
| ffplay form | windowed, **no** `-nodisp`, format auto | `-nodisp` recommended, `-input_format mjpeg` forced |
| format before open | none (ffplay does its own S_FMT) | `v4l2-ctl --set-fmt-video` first |
| pacing | full `SWITCH_INTERVAL`, `DURATION=300` | short cells |
| streaming | continuous for the whole phase | `setfmt` none, `stream` only 30 frames |

The three "improvements" that hid the bug were: `-nodisp` (no real capture),
a forced `-input_format mjpeg`, and the pre-emptive `v4l2-ctl --set-fmt-video`
(a clean S_FMT before ffplay opens can normalise the driver state). The current
`ffplay_switch_diag.sh` defaults to develop's ffplay form
(`SETFMT_BEFORE_OPEN=0`, no forced input format); setting
`SETFMT_BEFORE_OPEN=1` re-enables the pre-set and may make the fault harder to
hit.

### What `-nodisp` actually does

`ffplay -nodisp` means "disable graphical display". It is not a headless
capture mode: on a video-only stream (the back camera) ffplay 4.4 probes the
input, then fails to configure the video output/filtergraph and exits. It never
reaches a continuous read loop. Without `-nodisp`, ffplay with a display
creates a window and consumes frames continuously at real time; without a
display it aborts at SDL initialization before any capture. That is why the
two invocations behave so differently, and why the fault-injection script now
uses `setfmt`/`stream` for headless runs.

## Reproduce with the develop phase pattern (recommended)

```bash
cd <usb_cam>/scripts

# low+sidecars <-> high-solo every 2 s, up to 5 minutes
SWITCH_INTERVAL=2 DURATION=300 ./scripts/ffplay_oom_probe.sh

# stop earlier if only the switch itself is needed
SWITCH_INTERVAL=1 DURATION=60 ./scripts/ffplay_oom_probe.sh
```

Defaults follow develop: `SIDECARS="front left right"` (streamed only in the
low phase), `LOW_RES=640x480`, `HIGH_RES=1280x1024`, windowed ffplay, no
`-input_format`. The script checks sizeimage after every back-camera close and
exits **2** as soon as it is implausible, leaving the driver state for the next
`usb_cam` launch.

> `ffplay` must be able to open a window here. On a headless host use the
> `setfmt`/`stream` modes of `ffplay_switch_diag.sh` instead (see below); they
> do not reproduce the multi-camera phase pattern, but they are the only
> headless option.

## Reproduce with the diagnostic matrix

```bash
# on the robot host, from a checkout of the usb_cam sources
cd <usb_cam>/scripts

# pure v4l2-ctl format switching: 640x480 <-> 1280x1024, up to 50 switches
MODES=setfmt PER_INTERVAL_RUNS=50 PIXEL_FORMAT=MJPG \
  ./ffplay_switch_diag.sh

# same, plus 30 frames captured per resolution (headless, real REQBUFS)
MODES=stream PER_INTERVAL_RUNS=50 PIXEL_FORMAT=MJPG STREAM_COUNT=30 \
  ./ffplay_switch_diag.sh

# combined variant: windowed ffplay after each set-fmt (requires a display)
MODES=switch INTERVALS=2 PER_INTERVAL_RUNS=50 \
  ./ffplay_switch_diag.sh
```

Before each switch (and after every close, for the ffplay modes) the script runs

```bash
v4l2-ctl -d /dev/rgb_camera_back --get-fmt-video
```

and compares `Size Image` with `min(width * height * bpp * 4, 64 MiB)` (bpp:
MJPG 3, YUYV/UYVY 2, GREY 1). A value near 2^32 is also rejected. When the
value is implausible the script prints

```
  ABNORMAL SIZEIMAGE (after set-fmt 17/50 1280x1024 MJPG): sizeimage 4294967295 ...
  capture stopped on purpose; the driver state is left untouched.
```

and exits with code **2**. The device is closed but the driver keeps the
reported format, which is the state the next open must deal with.

Exit codes: `0` matrix clean, `1` leak/OOM findings, `2` abnormal sizeimage.

Tunables: `PIXEL_FORMAT` (default `MJPG`), `SETFMT_SETTLE` (default 0.5 s),
`STREAM_COUNT` (default 30, `stream` mode), `SIZEIMAGE_FACTOR` (default 4), `SIZEIMAGE_ABS_CAP` (default 67108864),
`V4L2CTL`, `STOP_ON_BAD_SIZEIMAGE` (default 1; set 0 to keep the
switch-only behaviour).

## Reproduce with usb_cam

Start the camera stack immediately after the script exits:

```bash
docker start <gpu-container>
# or, inside the container:
ros2 launch usb_cam rgb_camera_back_640x480.launch.py
```

What to look for:

```
[rgb_camera_back] dropping frame (...): sizeimage ... / mjpeg payload ... 
[rgb_camera_back] [FATAL] 10 consecutive bad frames ... container respawns
```

The first line comes from `frame_sanity_check`; the FATAL line from
`frame_sanity_max_consecutive` (default 10, `0` disables the escalation). The
container `respawn=True` restarts the whole stack ~10 s later.

If the fault does not reproduce, shorten `INTERVALS` (2, 1), raise
`PER_INTERVAL_RUNS`, or run the full matrix without `MODES=switch`; check
`dmesg` for `uvcvideo` errors as well.

## Recovery

After the experiment restart the camera stack (or the container). If the device
stays wedged, reset the USB port (`usbreset` / `uhubctl`) or replug the camera
and then restart the container.

See `frame_sanity.md` for the checks and the escalation parameters.
