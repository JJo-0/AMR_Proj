from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import os
def generate_launch_description():

    example_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('example_package'), 'launch'), '/example_launch_file.launch.py']
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([omo_r1mini_mcu_dir, '/omo_r1mini_mcu.launch.py']),
            launch_arguments={'omo_r1mini_mcu_parameter': omo_r1mini_mcu_parameter}.items()
        ),
    ])
