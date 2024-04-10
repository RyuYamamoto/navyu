from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  use_sim = LaunchConfiguration('use_sim')
  autostart = LaunchConfiguration('autostart')
  use_composition = LaunchConfiguration('use_composition')
  use_respawn = LaunchConfiguration('use_respawn')

  map_file_path = PathJoinSubstitution(
    [FindPackageShare('navyu_navigation'), 'map', 'map.yaml']
  )

  nav2_config_path = PathJoinSubstitution(
    [FindPackageShare('navyu_navigation'), 'config', 'nav2_params.yaml']
  )

  nav2_launch_path = PathJoinSubstitution(
    [FindPackageShare('nav2_bringup'), 'launch']
  )

  default_bt_xml_filename = PathJoinSubstitution(
    [FindPackageShare('nav2_bt_navigator'), 'behavior_trees', 'navigate_w_resplanning_and_recovery.xml']
  )

  rviz_config = PathJoinSubstitution(
    [FindPackageShare('navyu_navigation'), 'rviz', 'navigation.rviz']
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      'use_sim',
      default_value='true',
      description='Start robot in Gazebo simulation'),

    DeclareLaunchArgument(
      'autostart',
      default_value='true',
      description='Automatically startup the nav2 stack'),

    DeclareLaunchArgument(
      'use_composition',
      default_value='True',
      description='Whether to use composed bringup'),

    DeclareLaunchArgument(
      'use_respawn',
      default_value='false',
      description='Whether to respawn if a node crashes. \
      Applied when composition is disabled.'),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([nav2_launch_path, '/bringup_launch.py']),
      launch_arguments={
        'map': map_file_path,
        'use_sim_time': use_sim,
        'params_file': nav2_config_path,
        #'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        'use_composition': use_composition,
        'use_respawn': use_respawn
      }.items()
    ),

    Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d', rviz_config]
    )
  ])
