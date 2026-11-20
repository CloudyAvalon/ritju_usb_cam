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

import os
from pathlib import Path  # noqa: E402
import sys

# Hack to get relative import of .camera_config file working
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)

from camera_config import CameraConfig, USB_CAM_DIR  # noqa: E402
from apriltag_config import apriltag_component  # noqa: E402
from container_components import selected_components  # noqa: E402
from video_stream_config import image_stream_component  # noqa: E402

from launch import LaunchDescription  # noqa: E402
from launch_ros.actions import ComposableNodeContainer  # noqa: E402
from launch_ros.descriptions import ComposableNode  # noqa: E402

import json
def parse_cameras():
    # 从环境变量获取CAMERAS值
    cameras_str = os.getenv('CAMERAS')
    cameras_str = '[{"name":"rgb_camera_back", "config_file":"back_640x480.yaml"}]'

    if not cameras_str:
        print("错误：CAMERAS环境变量未设置")
        return None
    
    # 去除可能的单引号包裹
    if cameras_str and cameras_str.startswith("'") and cameras_str.endswith("'"):
        cameras_str = cameras_str[1:-1]
    
    # 解析JSON字符串
    try:
        # 解析JSON
        cameras_data = json.loads(cameras_str)
        
        # 验证数据结构
        if not isinstance(cameras_data, list):
            print("错误：CAMERAS不是一个列表")
            return None
            
        for camera in cameras_data:
            if not all(key in camera for key in ['name', 'config_file']):
                print(f"错误：相机配置缺少必要字段 - {camera}")
                return None
                
        return cameras_data
    except json.JSONDecodeError as e:
        print(f"JSON解析错误: {e}")
        return None

CAMERAS = []
# print(f"USB_CAM_DIR: {USB_CAM_DIR}")

camera_infos = parse_cameras()

if camera_infos != None:
    for i in range(len(camera_infos)):
        print(f'name: {camera_infos[i]["name"]}')
        print(f'config_file: {camera_infos[i]["config_file"]}')

    for i in range(len(camera_infos)):
    # for i in range(0,2):
        CAMERAS.append( 
            CameraConfig(
                name=camera_infos[i]["name"],
                param_path=Path('/map/config/usb_cam', camera_infos[i]["config_file"])
            )
        )

# CAMERAS.append(
#     CameraConfig(
#         name='camera1',
#         param_path=Path(USB_CAM_DIR, 'config', 'params_front.yaml')
#     )
#     # Add more Camera's here and they will automatically be launched below
# )

def generate_launch_description():
    ld = LaunchDescription()

    camera_nodes = [
        ComposableNode(
            package='usb_cam', plugin='usb_cam::UsbCamNode',
            name=camera.name,
            namespace=camera.namespace,
            parameters=[str(camera.param_path)],
            remappings=camera.remappings,
        )
        for camera in CAMERAS
    ]
    # Camera, recorder and charger detector share one container process: a
    # single launch entry, one process and one respawn policy cover them.  The
    # optional nodes are selected by USB_CAM_CONTAINER_COMPONENTS (camera,
    # camera+stream or camera+stream+apriltag).
    components = selected_components()
    if 'stream' in components:
        camera_nodes.append(image_stream_component())
    if 'apriltag' in components:
        camera_nodes.append(apriltag_component())
    print(f'[usb_cam] back-camera container components: {"+".join(components)}')

    ld.add_action(ComposableNodeContainer(
        name='usb_cam_with_image_stream_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=camera_nodes,
        output='screen',
        # respawn keeps camera and recorder alive across USB dropouts.
        respawn=True, respawn_delay=10,
    ))
    return ld
