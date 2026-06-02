from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg             = get_package_share_directory('mecanum_drive')
    params_file     = os.path.join(pkg, 'config', 'params.yaml')
    studica_params  = os.path.join(pkg, 'config', 'studica_params.yaml')
    studica_launch  = os.path.join(
        get_package_share_directory('studica_control'), 'launch', 'studica_launch.py')
    lidar_launch    = os.path.join(
        get_package_share_directory('lidar_merge'), 'launch', 'lidar_merge.launch.py')

    return LaunchDescription([

        # studica_control — VMX hardware driver + gamepad_component (/joy -> /cmd_vel)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(studica_launch),
            launch_arguments={'params_file': studica_params}.items(),
        ),

        # lidar_merge — both YDLidar drivers + merged /scan
        IncludeLaunchDescription(PythonLaunchDescriptionSource(lidar_launch)),

        # Raw DS4 input -> /joy
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[params_file],
        ),

        # /cmd_vel -> Titan motor duty cycle commands (holonomic kinematics)
        Node(
            package='mecanum_drive',
            executable='mecanum_drive_node',
            name='mecanum_drive',
            output='screen',
            parameters=[params_file],
        ),

        # encoder + IMU heading -> /odom (with lateral velocity)
        Node(
            package='mecanum_drive',
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

        # Static TF: base_link -> imu_link (132.41mm forward, 6.29mm left of base_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf',
            arguments=['0.13241', '0.00629', '0.1691', '0', '0', '0',
                       'base_link', 'imu_link'],
        ),
    ])
