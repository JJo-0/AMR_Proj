#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    

    driver_node = LifecycleNode(package='stella_ahrs',
                                node_executable='stella_ahrs_node',
                                node_name='stella_ahrs_node',
                                output='screen',
                                emulate_tty=True,
                                node_namespace='/',
                                )
    tf2_node = Node(package='tf2_ros',
                    node_executable='static_transform_publisher',
                    node_name='static_tf_pub_laser',
                    arguments=['0', '0', '0.02','0', '0', '0', '2','base_link','imu_link'],
                    )
                    
    return LaunchDescription([
      	driver_node,
        tf2_node,
    ])


