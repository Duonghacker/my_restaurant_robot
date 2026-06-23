"""
sim.launch.py
Package: restaurant_robot_description
Description: Legacy launch file - redirects to gazebo.launch.py.
             Kept for backward compatibility.

Usage:
  ros2 launch restaurant_robot_description sim.launch.py

For the full launch with all options, use:
  ros2 launch restaurant_robot_description gazebo.launch.py

ROS Version: ROS 2 Jazzy Jalisco
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_robot = get_package_share_directory('restaurant_robot_description')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_robot, 'launch', 'gazebo.launch.py')
            )
        )
    ])
