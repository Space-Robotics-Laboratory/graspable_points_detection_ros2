# Copyright (c) 2026 Space Robotics Lab -- Tohoku University
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('graspable_points_detection')
    rviz_config_path = os.path.join(
        pkg_path, 'config', 'test.rviz')

    rviz_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=str(rviz_config_path)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    graspable_points_detection_node = Node(
        package='graspable_points_detection',
        executable='graspable_points_detection_node',
        name='graspable_points_detection',
        output='screen'
    )

    test_params = os.path.join(pkg_path, 'config', 'test.yaml')

    point_cloud_publisher_node = Node(
        package='graspable_points_detection',
        executable='point_cloud_publisher_node',
        name='point_cloud_publisher',
        output='screen',
        parameters=[test_params]
    )

    return LaunchDescription([
        rviz_arg,
        rviz_node,
        graspable_points_detection_node,
        point_cloud_publisher_node
    ])
