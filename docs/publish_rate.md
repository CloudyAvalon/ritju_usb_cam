# Publish rate (`publish_rate`)

`publish_rate` decouples the **node publish/capture timer** from the **device
frame interval**:

```yaml
/**:
  ros__parameters:
    framerate: 15.0        # device interval (VIDIOC_S_PARM), if set_framerate
    publish_rate: 10.0     # timer period = 1 / publish_rate
```

- `publish_rate > 0` → timer period is `1000 / publish_rate` ms;
- **absent or `0`** → falls back to `framerate`, preserving the old behaviour;
- it is read from the same ROS parameter file as `framerate` (no separate
  config file);
- it does not touch `VIDIOC_S_PARM`; the device keeps its own rate.

Example behaviour with `framerate: 15` (device) and `publish_rate: 10`:

- device captures at 15 Hz;
- the timer wakes at 10 Hz and `read_frame()` drains all completed buffers,
  keeping only the newest one;
- ROS publishes at ~10 Hz, with latency about one device frame (66 ms) plus
  processing, instead of one publish period (100 ms).

This is useful with `set_framerate:=false` (device default rate) or when the
device rate must stay at 15 Hz while downstream consumers need 10 Hz.

## Caveats

- Changing `publish_rate` at runtime updates the parameter, but the existing
  timer period is not recreated; restart the node (or launch) to apply it.
- If `publish_rate` is higher than the device rate, the timer will often see
  no new frame and skip publishing; the effective rate is capped by the
  device.
- `publish_rate` must be finite, positive and ≤ 1000 Hz; invalid values make
  the node log an error and shut down.
