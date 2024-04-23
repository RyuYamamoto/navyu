from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    navyu_global_planner_config = PathJoinSubstitution(
        [FindPackageShare("navyu_planner"), "config", "navyu_global_planner_params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="navyu_planner",
                executable="navyu_global_planner_node",
                name="navyu_global_planner_node",
                output="screen",
                parameters=[navyu_global_planner_config, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
