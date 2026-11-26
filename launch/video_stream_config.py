"""Launch parameters for the cpp_video_stream image-recording component.

The back-camera launch files host cpp_video_stream's ``ImageStreamNode`` in
the same ``component_container_mt`` as ``usb_cam::UsbCamNode``: one launch
entry, one process and one respawn policy cover camera plus recorder.

Naming follows the deployed camera index: the back camera records as
``camera3`` (the on-robot video archive layout).  The environment overrides
mirror the ``container/video_stream`` branch defaults so both launch paths
stay interchangeable.
"""

import os

# Back-camera recording identity (matches /video_data/camera3 on the robot).
STREAM_NAME = 'camera3'
STREAM_ENABLE_TOPIC = 'camera3/stream_enable'
VIDEO_DIR = '/video_data/camera3/videos_cpp'


def get_image_stream_parameters():
    """Return the recording parameters used by the container/video_stream branch."""
    return {
        'stream_topic': os.environ.get(
            'TOPIC_CAMERA_BACK', 'rgb_camera_back/image_raw'),
        'is_compressed_image': (
            os.environ.get('TOPIC_CAMERA_BACK_IS_COMPRESSED', 'False') == 'True'
        ),
        'stream_enable_topic': STREAM_ENABLE_TOPIC,
        'stream_name': STREAM_NAME,
        'video_dir': VIDEO_DIR,
        'video_width': int(os.environ.get('VIDEO_WIDTH_BACK', '640')),
        'video_height': int(os.environ.get('VIDEO_HEIGHT_BACK', '480')),
        'disk_usage_threshold': float(
            os.environ.get('DISK_USAGE_THRESHOLD', '99.5')),
    }


def image_stream_component():
    """ComposableNode describing the back-camera recorder."""
    # Imported lazily so this module stays importable without launch_ros
    # (e.g. when only the parameter mapping is needed).
    from launch_ros.descriptions import ComposableNode

    return ComposableNode(
        package='cpp_video_stream',
        plugin='ImageStreamNode',
        name='cpp_image_stream_node_back',
        parameters=[get_image_stream_parameters()],
    )
