"""route_navigation.launch.py

Route-server based navigation for mecanum_drive_cpp.

Identical hardware stack to navigation.launch.py, but replaces free-space NavFn
planning with nav2_route's route_server, which follows a predefined GeoJSON graph.

Obstacle avoidance and dynamic re-routing are handled by the nav2_route::CollisionMonitor
operation inside route_server — it checks the global costmap every second and re-routes
through alternate graph edges when the planned path is blocked.

Usage:
  ros2 launch mecanum_drive_cpp route_navigation.launch.py \\
      map:=$HOME/maps/my_room.yaml \\
      graph:=$HOME/maps/my_graph.geojson

Build the graph first (once, with the map running):
  ros2 run graph_builder_cpp graph_builder \\
      --ros-args -p save_path:=$HOME/maps/my_graph.geojson

After launch:
  1. Open RViz → set "2D Pose Estimate" to robot's actual position.
  2. Send a "Nav2 Goal" — the robot follows graph edges to the nearest node
     to the goal, then drives the final segment to the goal pose.

Notes:
  - Gamepad is disabled automatically — forced off in a temp params copy at launch.
  - nav2_route must be installed: sudo apt install ros-humble-nav2-route
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import tempfile
import yaml


def generate_launch_description():
    pkg          = get_package_share_directory('mecanum_drive_cpp')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    lidar_pkg    = get_package_share_directory('lidar_merge_cpp')

    params_file    = os.path.join(pkg, 'config', 'params.yaml')
    bt_xml         = os.path.join(pkg, 'config', 'route_nav_bt.xml')
    nav2_params_yaml = os.path.join(pkg, 'config', 'route_nav2_params.yaml')

    # Patch route_nav2_params at launch time: inject the bt_xml absolute path
    # so the bt_navigator gets the route BT tree, not the default free-planning tree.
    with open(nav2_params_yaml, 'r') as f:
        _nav2_cfg = yaml.safe_load(f)
    _nav2_cfg['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = bt_xml
    _tmp_nav2 = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(_nav2_cfg, _tmp_nav2)
    _tmp_nav2.close()
    nav2_params = _tmp_nav2.name

    # Load studica_params and force gamepad off for navigation — Nav2 owns /cmd_vel.
    with open(os.path.join(pkg, 'config', 'studica_params.yaml'), 'r') as f:
        _studica_cfg = yaml.safe_load(f)
    _studica_cfg['control_server']['ros__parameters']['gamepad']['enabled'] = False
    _tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(_studica_cfg, _tmp)
    _tmp.close()
    studica_params = _tmp.name

    studica_launch = os.path.join(
        get_package_share_directory('studica_control'), 'launch', 'studica_launch.py')
    lidar_launch   = os.path.join(lidar_pkg, 'launch', 'lidar_merge.launch.py')
    nav2_launch    = os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')

    map_yaml   = LaunchConfiguration('map')
    graph_path = LaunchConfiguration('graph')

    return LaunchDescription([

        # Launch arguments

        DeclareLaunchArgument(
            'map',
            description='Full path to the map yaml (produced by mapping.launch.py). '
                        'Example: map:=$HOME/maps/my_room.yaml'
        ),

        DeclareLaunchArgument(
            'graph',
            description='Full path to the GeoJSON route graph '
                        '(produced by graph_builder_node). '
                        'Example: graph:=$HOME/maps/my_graph.geojson'
        ),

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

                # Route server
                Node(
                    package='nav2_route',
                    executable='route_server',
                    name='route_server',
                    output='screen',
                    parameters=[
                        nav2_params,
                        {'graph_filepath': graph_path},
                    ],
                ),

                # Route server lifecycle manager
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_route',
                    output='screen',
                    parameters=[
                        {'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['route_server']},
                    ],
                ),
            ],
        ),
    ])
