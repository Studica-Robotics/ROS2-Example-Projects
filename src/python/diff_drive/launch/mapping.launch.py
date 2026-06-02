from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg             = get_package_share_directory('diff_drive')
    lidar_pkg       = get_package_share_directory('lidar_merge_cpp')
    params_file     = os.path.join(pkg, 'config', 'params.yaml')
    slam_params     = os.path.join(pkg, 'config', 'slam_params.yaml')
    studica_params  = os.path.join(pkg, 'config', 'studica_params.yaml')
    studica_launch  = os.path.join(
        get_package_share_directory('studica_control'), 'launch', 'studica_launch.py')
    lidar_launch    = os.path.join(lidar_pkg, 'launch', 'lidar_merge.launch.py')

    return LaunchDescription([

        # studica_control — VMX hardware driver + gamepad_component (/joy -> /cmd_vel)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(studica_launch),
            launch_arguments={'params_file': studica_params}.items(),
        ),

        # lidar_merge_cpp — both YDLidar drivers + merged /scan
        # C++ version used for performance; Python lidar_merge is reference only.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(lidar_launch)),

        # Raw DS4 input -> /joy
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[params_file],
        ),

        # /cmd_vel -> Titan motor duty cycle commands
        Node(
            package='diff_drive',
            executable='diff_drive_node',
            name='diff_drive',
            output='screen',
            parameters=[params_file],
        ),

        # encoder + IMU heading -> /odom
        Node(
            package='diff_drive',
            executable='odometry_node',
            name='odometry',
            output='screen',
            parameters=[params_file],
        ),

        # EKF: /odom + /imu -> /odometry/filtered + TF odom -> base_link
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[params_file],
        ),

        # Static TF: base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf',
            arguments=['0.13241', '0.00629', '0.1691', '0', '0', '0',
                       'base_link', 'imu_link'],
        ),

        # Confidence scan gate: /scan -> [motion/overlap/ghost-wall filters] -> /scan_filtered
        # C++ node from lidar_merge_cpp — shared by all drive packages.
        # Config lives in lidar_merge_cpp — single source of truth for gate tuning.
        Node(
            package='lidar_merge_cpp',
            executable='confidence_scan_gate_node',
            name='confidence_scan_gate',
            output='screen',
            parameters=[os.path.join(lidar_pkg, 'config', 'confidence_gate_params.yaml')],
        ),

        # SLAM Toolbox: /scan_filtered + TF -> /map + TF map -> odom
        # Pinned to CPU core 3 (taskset) so SLAM's Ceres solver does not starve EKF/odometry.
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params],
            prefix='taskset -c 3',
        ),
    ])
