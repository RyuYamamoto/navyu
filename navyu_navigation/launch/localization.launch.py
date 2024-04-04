import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time')
  amcl_config_path = LaunchConfiguration('amcl_config_path')
  map_path = LaunchConfiguration('map_path')

  costmap_config_path = PathJoinSubstitution(
    [FindPackageShare('navyu_navigation'),
     'config', 'global_costmap_params.yaml']
  )

  use_sim_time_cmd = DeclareLaunchArgument(
    'use_sim_time', default_value='true', description=''
  )

  amcl_config_path_cmd = DeclareLaunchArgument(
    'amcl_config_path',
    default_value=PathJoinSubstitution([FindPackageShare('navyu_navigation'), 'config', 'amcl_params.yaml']),
    description=''
  )

  map_path_cmd = DeclareLaunchArgument(
    'map_path',
    default_value=PathJoinSubstitution([FindPackageShare('navyu_navigation'), 'map', 'map.yaml'])
  )

  amcl_node = Node(
    package='nav2_amcl',
    executable='amcl',
    name='amcl',
    output='screen',
    parameters=[amcl_config_path, {'use_sim_time': use_sim_time}],
  )

  map_server_node = Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    parameters=[{'yaml_filename': map_path}, {'frame_id': 'map'}]
  )
  
  lifecycle_node = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager',
    output='screen',
    emulate_tty=True,
    parameters=[{'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}]
  )

  rviz_config = os.path.join(get_package_share_directory('navyu_navigation'), 'rviz', 'navigation.rviz')
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config]
  )


  ld = LaunchDescription()

  ld.add_action(use_sim_time_cmd)
  ld.add_action(amcl_config_path_cmd)
  ld.add_action(map_path_cmd)
  ld.add_action(amcl_node)
  ld.add_action(map_server_node)
  ld.add_action(lifecycle_node)
  ld.add_action(rviz_node)

  return ld
