# Back-camera composable container: component selection

`rgb_camera_back_640x480.launch.py` and `rgb_camera_back_1280x1024.launch.py`
start one `component_container_mt` named `usb_cam_with_image_stream_container`.
Besides `usb_cam::UsbCamNode` it can host two optional nodes:

| component | node | provided by |
|---|---|---|
| `camera` (mandatory) | `rgb_camera_back` | `usb_cam` |
| `stream` (optional) | `cpp_image_stream_node_back` | `cpp_video_stream` |
| `apriltag` (optional) | `apriltag_double_node` | `apriltag_ros` |

The mix is selected by the environment variable
**`USB_CAM_CONTAINER_COMPONENTS`**:

| value | started nodes |
|---|---|
| `camera` | usb_cam only |
| `camera+stream` | usb_cam + cpp_video_stream |
| `camera+stream+apriltag` | all three (**default**, previous behaviour) |

`+` or `,` may separate tokens; `usb_cam`, `cpp_stream`, `image_stream`,
`cpp_video_stream` and `apriltag_ros` are accepted as aliases.
`camera` is mandatory; an empty/unknown value makes the launch fail with a
`ValueError` that lists the accepted tokens (fail fast instead of silently
starting a partial camera stack).

## Usage

`rgb_camera_manager` spawns these launches, so the variable is read from the
container environment:

```yaml
services:
  deep_learning:
    environment:
      - USB_CAM_CONTAINER_COMPONENTS=camera+stream
```

or, for a one-off run:

```bash
USB_CAM_CONTAINER_COMPONENTS=camera \
  ros2 launch usb_cam rgb_camera_back_640x480.launch.py
```

The launch prints the selected mix at startup:

```
[usb_cam] back-camera container components: camera+stream
```

## Effect of disabling nodes

- without `stream`: no recording under `/video_data/<stream_name>/videos_cpp`,
  and `/camera3/stream_enable` is not consumed;
- without `apriltag`: no `/start_detect_apriltag` service and no
  `/pose_with_id` output (docking must not rely on it on that robot).

Both back-camera launches share `launch/container_components.py`, so the
selection behaves identically at 640x480 and 1280x1024.

Driver frame validation and the consecutive-failure escalation are described in
[frame_sanity.md](frame_sanity.md).
