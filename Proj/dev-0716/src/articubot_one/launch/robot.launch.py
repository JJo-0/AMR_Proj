import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
def generate_launch_description():

    description = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("omo_r1mini_description"), 'launch', 'omo_r1mini_description.launch.py'
                )])
    )

    controller = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("omo_r1mini_bringup"), 'launch', 'omo_r1mini_mcu.launch.py'
                )])
    )
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller])

    imu = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("stella_ahrs"), 'launch', 'stella_ahrs_launch.py'
                )])
    )

    mapping = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("omo_r1mini_cartographer"), 'launch', 'cartographer.launch.py'
                )])
    )

    lidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ydlidar_ros2_driver"), 'launch', 'ydlidar_launch.py'
                )])
    )
    return LaunchDescription([
        description,
        lidar,
        delayed_controller_manager,
        imu,
        mapping
    ])
