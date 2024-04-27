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
from os import pathsep

import launch_ros
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_rviz = LaunchConfiguration("use_rviz", default="false")

    urdf_file = os.path.join(
        get_package_share_directory("navyu_simulator"), "urdf", "sample_robot.urdf"
    )
    world_file = os.path.join(
        get_package_share_directory("navyu_simulator"), "world", "willow_garage.world"
    )

    robot_description = launch_ros.descriptions.ParameterValue(
        Command(
            [
                "xacro",
                " ",
                urdf_file,
                " ",
                "use_ros2_control:=true",
                " ",
                "sim_mode:=true",
                " ",
                "gazebo:=true",
            ]
        ),
        value_type=str,
    )

    set_env_gazebo_resource = SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH",
        value=[
            EnvironmentVariable("GAZEBO_RESOURCE_PATH", default_value=""),
            pathsep,
            "/usr/share/gazebo-11",
            pathsep,
            PathJoinSubstitution([get_package_share_directory("navyu_simulator"), "world"]),
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}, {"use_sim_time": use_sim_time}],
    )

    world_file_path_arg = DeclareLaunchArgument(
        "world", default_value=world_file, description="gazebo world file"
    )

    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="false")

    gazebo_ros_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"),
        ),
    )

    spawn_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-topic",
            "/robot_description",
            "-entity",
            "sample_robot",
            "-x",
            "-9",
            "-y",
            "-17",
            "-z",
            "0.88",
            "-Y",
            "1.57",
        ],
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_drive_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    rviz_config = os.path.join(
        get_package_share_directory("navyu_simulator"), "rviz", "default.rviz"
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
    ld.add_action(world_file_path_arg)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_node)
    ld.add_action(gazebo_ros_cmd)
    ld.add_action(set_env_gazebo_resource)
    ld.add_action(joint_state_broadcaster_node)
    ld.add_action(diff_drive_controller_node)
    ld.add_action(rviz_node)

    return ld
