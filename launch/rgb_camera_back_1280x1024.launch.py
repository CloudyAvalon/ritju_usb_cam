# Copyright 2018 Lucas Walter
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Lucas Walter nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

_LAUNCH_DIR = str(Path(__file__).resolve().parent)
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)
from usb_cam_runtime_config import get_node_parameters
from apriltag_config import apriltag_component
from container_components import selected_components
from video_stream_config import image_stream_component


def _camera_container(context):
    dynamic = get_node_parameters('rgb_camera_back')
    explicit_rate = LaunchConfiguration('framerate').perform(context).strip()
    rate = float(explicit_rate) if explicit_rate else float(dynamic.get('framerate', 10.0))
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError('framerate must be finite and positive')
    width = int(LaunchConfiguration('image_width').perform(context))
    height = int(LaunchConfiguration('image_height').perform(context))
    if width <= 0 or height <= 0:
        raise ValueError('image dimensions must be positive')

    camera_node = ComposableNode(
        package='usb_cam', plugin='usb_cam::UsbCamNode',
        name='rgb_camera_back',
        parameters=[LaunchConfiguration('params_file').perform(context),
                    {'pub_compressed': False}, dynamic,
                    {'image_width': width, 'image_height': height, 'framerate': rate}],
        remappings=[('image_raw', 'rgb_camera_back/image_raw'),
                    ('image_raw/compressed', 'rgb_camera_back/compressed'),
                    ('image_raw/compressedDepth', 'rgb_camera_back/compressedDepth'),
                    ('image_raw/theora', 'rgb_camera_back/image_raw/theora'),
                    ('camera_info', 'rgb_camera_back/camera_info')],
    )
    # Camera, recorder and charger detector share one container process: a
    # single launch entry, one process and one respawn policy cover them.  The
    # optional nodes are selected by USB_CAM_CONTAINER_COMPONENTS (camera,
    # camera+stream or camera+stream+apriltag).
    components = selected_components()
    nodes = [camera_node]
    if 'stream' in components:
        nodes.append(image_stream_component())
    if 'apriltag' in components:
        nodes.append(apriltag_component())
    print(f'[usb_cam] back-camera container components: {"+".join(components)}')

    return [ComposableNodeContainer(
        name='usb_cam_with_image_stream_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=nodes,
        output='screen',
        # respawn keeps camera and recorder alive across USB dropouts.
        respawn=True, respawn_delay=20,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='/map/config/usb_cam/back_1280x1024.yaml'),
        DeclareLaunchArgument('image_width', default_value='1280'),
        DeclareLaunchArgument('image_height', default_value='1024'),
        DeclareLaunchArgument('framerate', default_value=''),
        OpaqueFunction(function=_camera_container),
    ])
