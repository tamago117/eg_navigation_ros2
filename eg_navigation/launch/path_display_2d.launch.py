# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = "eg_navigation"

    #args
    #filePath = LaunchConfiguration('filePath')
    filePath = os.path.join(get_package_share_directory(
        package_name), "config", "csv", "demo.csv")

    #parameters
    #csvFilePath = DeclareLaunchArgument('filePath', default_value=os.path.join(get_package_share_directory(
    #    package_name), "config", "csv", "demo.csv"))

    #config
    rviz_config = os.path.join(get_package_share_directory(
        package_name), "config", "rviz", "path_display_2d.rviz")

    #nodes
    wpLoad = Node(
            package='eg_wptool',
            node_executable='wpLoad',
            parameters=[{'loop_rate': 2},
                        {'filePath': filePath},
                        {'map_frame_id': "map"}],)

    wpVisualizer = Node(
            package='eg_wptool',
            node_executable='wpVisualizer',
            parameters=[{'loop_rate': 2},
                        {'markerSize': 3.0},],)

    rviz = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output={
                "stdout": "screen",
                "stderr": "log",
            },
        )

    return LaunchDescription([
        wpLoad,
        wpVisualizer,
        rviz,
        ])