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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    emcl2_config_path = LaunchConfiguration("emcl2_config_path")

    use_sim_time_cmd = DeclareLaunchArgument("use_sim_time", default_value="true", description="")

    emcl2_config_path_cmd = DeclareLaunchArgument(
        "emcl2_config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("navyu_navigation"), "config", "emcl2_params.yaml"]
        ),
        description="",
    )

    emcl2_node = Node(
        package="emcl2",
        executable="emcl2_node",
        name="emcl2_node",
        parameters=[emcl2_config_path, {"use_sim_time": use_sim_time}],
    )

    ld = LaunchDescription()

    ld.add_action(use_sim_time_cmd)
    ld.add_action(emcl2_config_path_cmd)
    ld.add_action(emcl2_node)

    return ld
