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

import launch_ros
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_rviz = LaunchConfiguration("use_rviz", default="true")

    urdf_file = os.path.join(
        get_package_share_directory("navyu_simulator"), "urdf", "sample_robot.urdf"
    )

    simple_simulator_config_path = os.path.join(
        get_package_share_directory("navyu_simulator"), "config", "simple_simulator_params.yaml"
    )

    robot_description = launch_ros.descriptions.ParameterValue(
        Command(["xacro", " ", urdf_file]), value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}, {"use_sim_time": use_sim_time}],
    )

    simple_simulator_node = Node(
        package="navyu_simulator",
        executable="navyu_simulator_node",
        # name='navyu_simulator_node',
        output="screen",
        parameters=[simple_simulator_config_path, {"use_sim_time": use_sim_time}],
    )

    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")

    rviz_config = os.path.join(
        get_package_share_directory("navyu_simulator"), "rviz", "simple_simulator.rviz"
    )
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
    )

    ld = LaunchDescription()

    ld.add_action(use_rviz_arg)
    ld.add_action(simple_simulator_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz_node)

    return ld
