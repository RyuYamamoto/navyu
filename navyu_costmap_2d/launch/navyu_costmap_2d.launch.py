# Copyright 2024 RyuYamamoto.
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
# limitations under the License

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("navyu_costmap_2d"), "rviz", "costmap_test.rviz"]
    )

    costmap_map_config_path = PathJoinSubstitution(
        [FindPackageShare("navyu_costmap_2d"), "config", "navyu_costmap_2d_params.yaml"]
    )

    map_path = PathJoinSubstitution([FindPackageShare("navyu_navigation"), "map", "map.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="navyu_costmap_2d",
                executable="navyu_costmap_2d_node",
                name="global_costmap_node",
                output="screen",
                parameters=[costmap_map_config_path, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[{"yaml_filename": map_path}, {"frame_id": "map"}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="map_server_lifecycle_manager",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"use_sim_time": True},
                    {"autostart": True},
                    {"node_names": ["map_server"]},
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_config_path],
            ),
        ]
    )