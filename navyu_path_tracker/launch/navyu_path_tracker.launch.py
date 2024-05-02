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

    navyu_path_tracker_config = PathJoinSubstitution(
        [FindPackageShare("navyu_path_tracker"), "config", "navyu_path_tracker_params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="navyu_path_tracker",
                executable="navyu_path_tracker_node",
                name="navyu_path_tracker_node",
                output="screen",
                parameters=[navyu_path_tracker_config, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
