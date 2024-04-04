from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

  costmap_config_path = PathJoinSubstitution(
    [FindPackageShare('navyu_navigation'),
     'config', 'global_costmap_params.yaml']
  )

  map_path = PathJoinSubstitution(
    [FindPackageShare('navyu_navigation'),
     'map', 'map.yaml']
  )

  return LaunchDescription([
    #Node(
    #  package='nav2_map_server',
    #  executable='map_server',
    #  name='map_server',
    #  parameters=[{'yaml_filename': map_path},
    #              {'frame_id': 'map'}]
    #),
    Node(
      package='nav2_costmap_2d',
      executable='nav2_costmap_2d',
      name='costmap',
      output='screen',
      parameters=[costmap_config_path]
    ),

    Node(
      package='nav2_lifecycle_manager',
      executable='lifecycle_manager',
      name='map_server_lifecycle_manager',
      output='screen',
      emulate_tty=True,
      parameters=[{'use_sim_time': True},
                  {'autostart': True},
                  {'node_names': ['costmap']}]
    )
  ])
