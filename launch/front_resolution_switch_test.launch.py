# Copyright 2026 usb_cam Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Repeatedly restart the front usb_cam node at two resolutions.

Each cycle starts ``usb_cam_node_exe`` with one of the two geometries and stops
it after ``interval`` seconds. This exercises the same open/close path as the
deployed resolution switch, but with the real camera node, so frame sanity
drops and the consecutive-failure escalation show up directly in the node log.

Examples:
  ros2 launch usb_cam front_resolution_switch_test.launch.py
  ros2 launch usb_cam front_resolution_switch_test.launch.py \
      interval:=6 high_res:=1280x1024 cycles:=20
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('usb_cam'), 'config', 'front.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='ROS parameter file for the front camera'),
        DeclareLaunchArgument(
            'device', default_value='/dev/rgb_camera_front',
            description='V4L2 device of the front camera'),
        DeclareLaunchArgument(
            'node_name', default_value='rgb_camera_front',
            description='ROS node name used for each instance'),
        DeclareLaunchArgument(
            'low_res', default_value='640x480',
            description='first resolution of the alternation'),
        DeclareLaunchArgument(
            'high_res', default_value='1280x1024',
            description='second resolution of the alternation'),
        DeclareLaunchArgument(
            'interval', default_value='8.0',
            description='seconds each node instance stays up'),
        DeclareLaunchArgument(
            'cycles', default_value='0',
            description='number of restarts, 0 = until Ctrl-C'),
        DeclareLaunchArgument(
            'pixel_format', default_value='',
            description='override pixel_format (default: params file)'),
        DeclareLaunchArgument(
            'check_sizeimage', default_value='true',
            description='query v4l2-ctl after each stop and stop on an '
                        'implausible Size Image'),
        DeclareLaunchArgument(
            'sizeimage_stop_mib', default_value='100',
            description='stop switching when Size Image reaches this many MiB'),
        DeclareLaunchArgument(
            'io_method', default_value='',
            description='override io_method (e.g. userptr for a safe test)'),
        DeclareLaunchArgument(
            'pub_raw', default_value='true',
            description='publish <node_name>/image_raw (test default true)'),
        DeclareLaunchArgument(
            'pub_compressed', default_value='true',
            description='publish <node_name>/compressed'),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'usb_cam', 'usb_cam_resolution_switch_test',
                '--params-file', LaunchConfiguration('params_file'),
                '--device', LaunchConfiguration('device'),
                '--node-name', LaunchConfiguration('node_name'),
                '--low', LaunchConfiguration('low_res'),
                '--high', LaunchConfiguration('high_res'),
                '--interval', LaunchConfiguration('interval'),
                '--cycles', LaunchConfiguration('cycles'),
                '--pixel-format', LaunchConfiguration('pixel_format'),
                '--check-sizeimage', LaunchConfiguration('check_sizeimage'),
                '--sizeimage-stop-mib',
                LaunchConfiguration('sizeimage_stop_mib'),
                '--io-method', LaunchConfiguration('io_method'),
                '--pub-raw', LaunchConfiguration('pub_raw'),
                '--pub-compressed', LaunchConfiguration('pub_compressed'),
            ],
            output='screen',
        ),
    ])
