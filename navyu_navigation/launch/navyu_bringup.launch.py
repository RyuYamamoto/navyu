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

from launch import LaunchDescription, condition
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    map_path = PathJoinSubstitution([FindPackageShare("navyu_navigation"), "map", "map.yaml"])
    rviz_config = PathJoinSubstitution([FindPackageShare("navyu_navigation"), "rviz", "navyu.rviz"])

    navyu_config_path = PathJoinSubstitution(
        [FindPackageShare("navyu_navigation"), "config", "navyu_params.yaml"]
    )

    lifecycle_node_list = ["map_server"]

    navyu_launch_path = PathJoinSubstitution([FindPackageShare("navyu_navigation"), "launch"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[{"yaml_filename": map_path}],
            ),
            Node(
                package="navyu_costmap_2d",
                executable="navyu_costmap_2d_node",
                name="navyu_global_costmap_node",
                remappings=[("/costmap", "/global_costmap")],
                parameters=[navyu_config_path, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="navyu_costmap_2d",
                executable="navyu_costmap_2d_node",
                name="navyu_local_costmap_node",
                remappings=[("/costmap", "/local_costmap")],
                parameters=[navyu_config_path, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="navyu_path_planner",
                executable="navyu_global_planner_node",
                name="navyu_global_planner_node",
                remappings=[("/costmap", "/global_costmap")],
                parameters=[navyu_config_path, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="navyu_path_tracker",
                executable="navyu_path_tracker_node",
                name="navyu_path_tracker_node",
                remappings=[("/cmd_vel", "/cmd_vel_in")],
                parameters=[navyu_config_path, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="navyu_safety_limiter",
                executable="navyu_safety_limiter_node",
                name="navyu_safety_limiter_node",
                remappings=[("/cmd_vel_out", "/cmd_vel")],
                parameters=[navyu_config_path, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    {"node_names": lifecycle_node_list},
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([navyu_launch_path, "/emcl2.launch.py"]),
                condition=IfCondition(LaunchConfiguration("localization")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
