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
    delayed_controller_manager = TimerAction(period=6.0, actions=[controller])

    imu = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("stella_ahrs"), 'launch', 'stella_ahrs_launch.py'
                )])
    )


    lidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ydlidar_ros2_driver"), 'launch', 'ydlidar_launch.py'
                )])
    )
    delayed_description_manager = TimerAction(period=3.0, actions=[description])
    navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("omo_r1mini_navigation2"), 'launch', 'navigation2.launch.py'
                )])
    )
    delayed_navigation_manager = TimerAction(period=9.0, actions=[navigation])

    mapping = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("articubot_one"), 'launch', 'online_async_launch.py'
                )])
    )
    cartographer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("omo_r1mini_cartographer"), 'launch', 'cartographer.launch.py'
                )])
    )
    delayed_cartographer_manager = TimerAction(period=9.0, actions=[cartographer])


    return LaunchDescription([
        imu,
        lidar,
        delayed_controller_manager,
        delayed_description_manager,
        # delayed_navigation_manager,
        # delayed_rviz_manager,
        # mapping,
        # delayed_cartographer_manager,
    ])
