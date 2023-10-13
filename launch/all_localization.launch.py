import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'my_bot'  # <--- CHANGE ME

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rplidar.launch.py'
        )])
    )

    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'launch_robot.launch.py'
        )])
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'navigation_launch.py'
        )])
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'localization_launch.py'
        )])
    )

    # Define timer actions to introduce delays for each launch file
    timer_rplidar = TimerAction(
        period=0.0,  # Delay for 5 seconds before rplidar launch
        actions=[rplidar]
    )

    timer_launch_robot = TimerAction(
        period=5.0,  # Delay for 5 seconds before launch_robot launch
        actions=[launch_robot]
    )

    timer_navigation_launch = TimerAction(
        period=16.0,  # Delay for 5 seconds before navigation_launch launch
        actions=[navigation_launch]
    )

    timer_localization_launch = TimerAction(
        period=28.0,  # Delay for 5 seconds before localization_launch launch
        actions=[localization_launch]
    )

    # Launch them one at a time with timers in between
    return LaunchDescription([
        timer_rplidar,
        timer_launch_robot,
        timer_navigation_launch,
        timer_localization_launch,
    ])
