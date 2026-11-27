# Driver frame sanity checks and failure escalation

`usb_cam` validates every dequeued V4L2 buffer before it is decoded/published.
The checks are cheap (`O(1)` on the payload header) and aim to keep a broken
driver from either crashing the camera container or feeding garbage to the
perception stack.

## What is checked

| check | trigger | action |
|---|---|---|
| payload length | `v4l2_buffer.bytesused == 0` or larger than the buffer capacity | drop the frame, do not decode |
| MJPEG SOI | MJPEG payload does not start with `FF D8` | drop the frame, do not decode |
| decode failure | `process_image()` throws (corrupt MJPEG, ...) | drop the frame, keep the node alive |

A dropped frame is **not published** (the previous image is not re-sent), so a
bad frame produces a gap instead of stale/garbage data. Each drop increments a
total and a consecutive counter; the reason is logged on the first bad frame
and every 10 consecutive ones.

## Parameters

| parameter | default | meaning |
|---|---|---|
| `frame_sanity_check` | `true` | enable the payload / SOI pre-checks. Decode failures are always caught and dropped, even when this is `false`. |
| `frame_sanity_max_consecutive` | `10` | after this many consecutive dropped frames the node shuts down so the container respawns. `0` disables the escalation (log only). |

At the default 10 Hz capture rate, 10 consecutive bad frames is about one
second of continuous failure; transient one-off glitches reset the counter and
never trigger the escalation.

## Escalation

When the threshold is reached the node logs

```
[FATAL] ... 10 consecutive bad frames (last: mjpeg payload missing SOI marker);
shutting the camera stack down so the container respawns
```

and calls `rclcpp::shutdown()`. The back-camera launch starts the composable
container with `respawn=True, respawn_delay=10`, so the whole stack
(`usb_cam`, `cpp_video_stream`, `apriltag_ros`) is restarted about 10 s later.
This makes a persistent driver failure visible in the logs and in the container
restart count instead of silently publishing nothing forever.

Because the three nodes share one process, shutting down `usb_cam` also stops
the recorder and the charger detector for those ~10 s. Robots that cannot
tolerate that interruption should raise `frame_sanity_max_consecutive` (or set
it to `0` and monitor the `dropping frame` logs / call
`/rgb_camera_back/set_capture` manually).
