#!/usr/bin/env python3
"""
rviz2_remote.launch.py — Chạy trên LAPTOP Dell Vostro
Chỉ mở RViz2 để visualize dữ liệu từ Pi 4 qua mạng Wi-Fi.

Yêu cầu: cùng ROS_DOMAIN_ID với Pi 4 (mặc định=0)
  export ROS_DOMAIN_ID=0

Lệnh chạy trên Laptop:
  source ~/dev_ws/install/setup.bash
  ros2 launch restaurant_robot_bringup rviz2_remote.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('restaurant_robot_bringup')

    mode_arg = DeclareLaunchArgument(
        'mode', default_value='mapping',
        description='mapping | navigation — chọn config RViz2 phù hợp',
    )

    rviz_config = os.path.join(bringup_dir, 'config', 'mapping_rviz.rviz')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
    )

    return LaunchDescription([
        mode_arg,
        rviz2,
    ])
