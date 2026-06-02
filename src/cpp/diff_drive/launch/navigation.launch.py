"""navigation.launch.py

Full autonomous navigation stack for diff_drive_cpp:

  Hardware layer    studica_control (Titan motors + IMU + gamepad)
  Sensor layer      lidar_merge_cpp (both YDLidars -> /scan)
  Scan filter       confidence_scan_gate_node (/scan -> /scan_filtered)
  Drive             diff_drive_node (/cmd_vel -> Titan duty cycle)
  Odometry          odometry_node (encoders + IMU yaw -> /odom)
  State estimation  ekf_node (/odom + /imu -> /odometry/filtered + TF odom->base_link)
  Localization      AMCL  (pgm map + /scan_filtered -> TF map->odom)
  Navigation        Nav2  (planner + RPP controller + behaviours + costmaps)

Usage:
  ros2 launch diff_drive_cpp navigation.launch.py map:=<path-to-map.yaml>

  # Example (map saved during previous mapping session):
  ros2 launch diff_drive_cpp navigation.launch.py \\
      map:=$HOME/maps/my_map.yaml

After launch:
  1. Open RViz on your laptop/VM
  2. Click "2D Pose Estimate" in the RViz toolbar.
  3. Click + drag on the map to set the robot's starting position and heading.
     AMCL will not move without an initial pose.
  4. Click "Nav2 Goal" (or "2D Goal Pose") in the RViz toolbar.
  5. Click + drag on the map to send a navigation goal.

Notes:
  - Gamepad is disabled automatically — this launch forces gamepad: enabled: false
    in a temp copy of studica_params.yaml so Nav2 owns /cmd_vel without conflict.
  - Nav2 uses /odometry/filtered (EKF output), not raw /odom.
  - The confidence scan gate uses /scan_filtered for AMCL.
    Costmaps subscribe to /scan (raw merged scan) so obstacles are never
    missed during robot motion (the motion gate would filter them out).
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
    pkg             = get_package_share_directory('diff_drive_cpp')
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

        DeclareLaunchArgument(
            'map',
            description='Full path to the map yaml file produced by mapping.launch.py. '
                        'Example: map:=$HOME/maps/my_map.yaml'
        )

        # Hardware driver
        # Starts: Titan CAN driver, IMU.
        # Gamepad is disabled automatically (forced off in the temp params above).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(studica_launch),
            launch_arguments={'params_file': studica_params}.items(),
        ),

        # Lidar stack
        # Starts both YDLidar drivers and merges them into /scan.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(lidar_launch)),

        # Drive node
        # /cmd_vel (from Nav2 controller_server) -> Titan motor duty cycle
        Node(
            package='diff_drive_cpp',
            executable='diff_drive_node',
            name='diff_drive',
            output='screen',
            parameters=[params_file],
        ),

        # Odometry
        # Encoder distance deltas + IMU yaw -> /odom
        Node(
            package='diff_drive_cpp',
            executable='odometry_node',
            name='odometry',
            output='screen',
            parameters=[params_file],
        ),

        # EKF
        # /odom + /imu -> /odometry/filtered + TF odom -> base_link
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[params_file],
        ),

        # Static TFs
        # base_link -> imu_link  (132.41mm forward, 6.29mm left of base_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf',
            arguments=['0.13241', '0.00629', '0.1691', '0', '0', '0',
                       'base_link', 'imu_link'],
        ),

        # Nav2 bringup
        # Delayed 5 s to let both YDLidar Tmini Plus units finish their
        # intensity auto-calibration before Nav2 starts loading its shared
        # libraries.  Loading 10+ nav2 .so files on the Pi while the
        # lidars are mid-calibration causes a driver segfault (exit -11).
        #
        # Starts all Nav2 nodes via lifecycle managers:
        #   map_server  — serves the pgm/yaml map
        #   amcl        — localises robot in the map using /scan_filtered
        #   bt_navigator, planner_server, controller_server, behavior_server
        #   waypoint_follower, smoother_server
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
