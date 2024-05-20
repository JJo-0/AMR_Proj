import os
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():

    return LaunchDescription([
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyLidar',
                'frame_id': 'base_scan',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        )
    ])
