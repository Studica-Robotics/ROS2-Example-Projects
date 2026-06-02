"""navigation.launch.py

Full autonomous navigation stack for mecanum_drive_cpp:

  Hardware layer    studica_control (Titan motors + IMU + gamepad)
  Sensor layer      lidar_merge_cpp (both YDLidars -> /scan)
  Drive             mecanum_drive_node (/cmd_vel -> Titan duty cycle, holonomic)
  Odometry          odometry_node (encoders + IMU yaw -> /odom)
  State estimation  ekf_node (/odom + /imu -> /odometry/filtered + TF odom->base_link)
  Localization      AMCL  (pgm map + /scan -> TF map->odom)
  Navigation        Nav2  (planner + RPP controller + behaviours + costmaps)

Usage:
  ros2 launch mecanum_drive_cpp navigation.launch.py map:=<path-to-map.yaml>

After launch:
  1. Open RViz on your laptop/VM
  2. Click "2D Pose Estimate" in the RViz toolbar.
  3. Click + drag on the map to set the robot's starting position and heading.
     AMCL will not move without an initial pose.
  4. Click "Nav2 Goal" in the RViz toolbar.
  5. Click + drag on the map to send a navigation goal.

Notes:
  - Gamepad is disabled automatically — this launch forces gamepad: enabled: false
    in a temp copy of studica_params.yaml so Nav2 owns /cmd_vel without conflict.
  - Nav2 uses /odometry/filtered (EKF output), not raw /odom.
  - AMCL subscribes to /scan (raw merged scan) — not /scan_filtered.
    The confidence gate's motion filter starves AMCL during turns (see CLAUDE.md).
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
    pkg             = get_package_share_directory('mecanum_drive_cpp')
    nav2_bringup    = get_package_share_directory('nav2_bringup')
    lidar_pkg       = get_package_share_directory('lidar_merge_cpp')

    params_file    = os.path.join(pkg,       'config', 'params.yaml')
    nav2_params    = os.path.join(pkg,       'config', 'nav2_params.yaml')

    # Load studica_params and force gamepad off for navigation — Nav2 owns /cmd_vel.
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

        # Launch arguments

        DeclareLaunchArgument(
            'map',
            description='Full path to the map yaml file produced by mapping.launch.py. '
                        'Example: map:=$HOME/maps/my_map.yaml'
        )

        # Hardware driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(studica_launch),
            launch_arguments={'params_file': studica_params}.items(),
        ),

        # Lidar stack
        IncludeLaunchDescription(PythonLaunchDescriptionSource(lidar_launch)),

        # Drive node
        Node(
            package='mecanum_drive_cpp',
            executable='mecanum_drive_node',
            name='mecanum_drive',
            output='screen',
            parameters=[params_file],
        ),

        # Odometry
        Node(
            package='mecanum_drive_cpp',
            executable='odometry_node',
            name='odometry',
            output='screen',
            parameters=[params_file],
        ),

        # EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[params_file],
        ),

        # Static TFs
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf',
            arguments=['0.13241', '0.00629', '0.1691', '0', '0', '0',
                       'base_link', 'imu_link'],
        ),

        # Nav2 bringup
        # Delayed 5 s to let both YDLidar Tmini Plus units finish their
        # intensity auto-calibration before Nav2 starts loading shared libraries.
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
        ),
    ])
