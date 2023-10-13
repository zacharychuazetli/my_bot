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

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

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

    online_async_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'online_async_launch.py'
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

    timer_online_async_launch = TimerAction(
        period=16.0,  # Delay for 5 seconds before online_async_launch launch
        actions=[online_async_launch]
    )

    # Launch them one at a time with timers in between
    return LaunchDescription([
        timer_rplidar,
        timer_launch_robot,
        timer_online_async_launch,
    ])
