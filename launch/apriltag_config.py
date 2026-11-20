"""Launch parameters for the apriltag_ros charger detector.

The detector consumes the back camera's ``image_raw`` / ``camera_info``, so it
is loaded into the same ``component_container_mt`` as ``usb_cam::UsbCamNode``:
it starts and stops together with the back camera (``rgb_camera_manager``
restarts this launch when the resolution mode changes).

Loading the node does not start detection: the docking flow calls
``/start_detect_apriltag`` and ``/stop_detect_apriltag``, and the node handles
missing images by timing out.  No extra gating is needed here.

The parameter file lives under ``/map/config`` and the overrides mirror
``capella_ros_launcher/launch/apriltag_detection.launch.py`` so both entry
points stay interchangeable.

Requires ``apriltag_ros`` to be installed (the GPU image provides it).  It is
deliberately not declared in ``package.xml``: it is a workspace package, so an
``exec_depend`` would force colcon to build apriltag/apriltag_ros whenever
usb_cam is built on its own.
"""

import os
import shutil

# Parameter file seeded by the GPU image under /map/config.
CONF_DIR = os.environ.get('APRILTAG_DETECTION_CONF_DIR', '/map/config/apriltag_ros')
PARAMS_FILE = os.path.join(CONF_DIR, 'apriltag_double.yaml')


def _default_params_file():
    """Package default used as the seed source (None if apriltag_ros is absent)."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(
            get_package_share_directory('apriltag_ros'), 'cfg', 'apriltag_double.yaml')
    except Exception as exc:  # PackageNotFoundError / import error
        print(f'[apriltag] cannot locate the apriltag_ros package default: {exc}')
        return None


def ensure_params_file():
    """Return a params file that exists, seeding /map/config on first start.

    ``ComposableNode(parameters=[PARAMS_FILE, ...])`` makes launch read the file
    while the component is created.  /map is a bind mount, so on a fresh robot
    the file is absent and the whole camera launch dies with::

        Caught exception in launch: [Errno 2] No such file or directory:
        '/map/config/apriltag_ros/apriltag_double.yaml'

    Seed it from the package default instead.  If /map is missing or read-only,
    fall back to the package default so a camera problem cannot take down the
    camera container.
    """
    if os.path.isfile(PARAMS_FILE):
        return PARAMS_FILE

    default_file = _default_params_file()
    if default_file is None:
        return PARAMS_FILE  # nothing to seed from: let launch report the missing file

    try:
        os.makedirs(CONF_DIR, exist_ok=True)
        shutil.copy2(default_file, PARAMS_FILE)
        print(f'[apriltag] seeded {PARAMS_FILE} from {default_file}')
        return PARAMS_FILE
    except OSError as exc:
        print(
            f'[apriltag] WARNING: cannot seed {PARAMS_FILE} ({exc}); '
            f'using package default {default_file}')
        return default_file


def _env_str(name, default):
    value = os.environ.get(name, '')
    return value if value.strip() else default


def _env_float(name, default):
    raw = os.environ.get(name, '')
    if not raw.strip():
        return default
    try:
        return float(raw)
    except ValueError:
        print(f'[apriltag] {name}={raw!r} is not a float, using {default}')
        return default


def get_apriltag_parameters():
    """Return the launch-injected parameters (yaml file values are overwritten)."""
    marker_map = _env_str(
        'marker_id_and_bluetooth_mac', '0:1/94:C9:60:43:BE:12')
    return {
        'marker_id_and_bluetooth_mac_vec': [
            item.strip() for item in marker_map.split(',') if item.strip()
        ],
        'size': _env_float('APRILTAG_SIZE', 0.06),
        'marker_frame_translation': _env_float('MARKER_FRAME_TRANSLATION', -0.05),
        'similarity_threshold': _env_float('SIMILARITY_THRESHOLD', 0.99),
        'radius_threshold': _env_float('MARKER_FRAME_RADIUS_THRESHOLD', 0.017),
        'base_link_dummy_transform_x': _env_float('BASE_LINK_DUMMY_TRANSFORM_X', -0.573),
        'base_link_dummy_transform_y': _env_float('BASE_LINK_DUMMY_TRANSFORM_Y', 0.0),
        'base_link_dummy_transform_z': _env_float('BASE_LINK_DUMMY_TRANSFORM_Z', 0.477),
    }


def apriltag_component():
    """ComposableNode describing the back-camera charger detector."""
    # Imported lazily so this module stays importable without launch_ros.
    from launch_ros.descriptions import ComposableNode

    image_topic = _env_str(
        'APRILTAG_DOUBLE_IMAGE_TOPIC', '/rgb_camera_back/image_raw')
    camera_info_topic = _env_str(
        'APRILTAG_DOUBLE_CAMERA_INFO_TOPIC', '/rgb_camera_back/camera_info')

    return ComposableNode(
        package='apriltag_ros',
        plugin='AprilTagDoubleNode',
        name='apriltag_double_node',
        parameters=[ensure_params_file(), get_apriltag_parameters()],
        remappings=[('/image_rect', image_topic),
                    ('/camera_info', camera_info_topic)],
    )
