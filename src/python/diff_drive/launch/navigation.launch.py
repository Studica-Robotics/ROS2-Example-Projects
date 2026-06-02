"""navigation.launch.py — Python diff_drive (reference implementation)

Full autonomous navigation stack. Mirrors diff_drive_cpp/launch/navigation.launch.py.
C++ package is preferred for deployment; this is kept as a readable reference.

Usage:
  ros2 launch diff_drive navigation.launch.py map:=$HOME/maps/my_map.yaml
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import tempfile
import yaml


def generate_launch_description():
    pkg             = get_package_share_directory('diff_drive')
    nav2_bringup    = get_package_share_directory('nav2_bringup')
    lidar_pkg       = get_package_share_directory('lidar_merge_cpp')

    params_file    = os.path.join(pkg, 'config', 'params.yaml')
    nav2_params    = os.path.join(pkg, 'config', 'nav2_params.yaml')

    with open(os.path.join(pkg, 'config', 'studica_params.yaml'), 'r') as f:
        _studica_cfg = yaml.safe_load(f)
    _studica_cfg['control_server']['ros__parameters']['gamepad']['enabled'] = False
    _tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(_studica_cfg, _tmp)
    _tmp.close()
    studica_params = _tmp.name

    studica_launch  = os.path.join(
        get_package_share_directory('studica_control'), 'launch', 'studica_launch.py')
    lidar_launch    = os.path.join(lidar_pkg, 'launch', 'lidar_merge.launch.py')
    nav2_launch     = os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')

    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([

        DeclareLaunchArgument('map', description='Path to map yaml.'),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(studica_launch),
            launch_arguments={'params_file': studica_params}.items(),
        ),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(lidar_launch)),

        Node(
            package='diff_drive',
            executable='diff_drive_node',
            name='diff_drive',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='diff_drive',
            executable='odometry_node',
            name='odometry',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf',
            arguments=['0.13241', '0.00629', '0.1691', '0', '0', '0',
                       'base_link', 'imu_link'],
        ),

        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch),
                    launch_arguments={
                        'map':          map_yaml,
                        'use_sim_time': 'false',
                        'params_file':  nav2_params,
                        'autostart':    'true',
                    }.items(),
                ),
            ],
        )
    ])
