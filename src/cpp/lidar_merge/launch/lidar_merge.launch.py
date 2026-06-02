from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import math


def generate_launch_description():
    pkg = get_package_share_directory('lidar_merge_cpp')

    return LaunchDescription([

        # Front YDLidar driver — scan remapped to /front/scan
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='front_lidar',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'front_lidar.yaml')],
            remappings=[('/scan', '/front/scan')],
        ),

        # Back YDLidar driver — scan remapped to /back/scan
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='back_lidar',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'back_lidar.yaml')],
            remappings=[('/scan', '/back/scan')],
        ),

        # Static TF: base_link -> front_lidar (yaw = 0, zero faces outward/forward)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_front_lidar',
            arguments=['0.19195', '0', '0.1083',
                       '0', '0', '0',
                       'base_link', 'front_lidar'],
        ),

        # Static TF: base_link -> back_lidar (yaw = π, zero faces outward/backward)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_back_lidar',
            arguments=['-0.19195', '0', '0.1083',
                       str(math.pi), '0', '0',
                       'base_link', 'back_lidar'],
        ),

        # Static TF: base_link -> base_scan
        # Places the merged scan at the same height as the physical lidars (0.1083 m).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_scan',
            arguments=['0', '0', '0.1083',
                       '0', '0', '0',
                       'base_link', 'base_scan'],
        ),

        # Merge node: /front/scan + /back/scan -> /scan
        Node(
            package='lidar_merge_cpp',
            executable='lidar_merge_node',
            name='lidar_merge',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'params.yaml')],
        ),
    ])
