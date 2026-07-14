#!/usr/bin/env python3
"""
bringup.launch.py — Phase 1: MAPPING
Chạy trên Raspberry Pi 4 (Ubuntu 24.04, ROS 2 Jazzy, ros-base)

Khởi động:
  1. STM32 bridge (odom + cmd_vel)
  2. LiDAR driver (Camsense X2M)
  3. Static TF: base_link → laser_frame
  4. SLAM Toolbox (online async mapping)

Lệnh chạy trên Pi 4:
  source ~/dev_ws/install/setup.bash
  ros2 launch restaurant_robot_bringup bringup.launch.py

Sau khi mapping xong, lưu map:
  ros2 run nav2_map_server map_saver_cli -f ~/map/restaurant_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    # ── Package dirs ──────────────────────────────
    bringup_dir = get_package_share_directory('restaurant_robot_bringup')
    lidar_dir   = get_package_share_directory('hclidar_driver_ros2')

    # ── Launch arguments ──────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyAMA0',
        description='UART port kết nối STM32 (GPIO 14/15 trên Pi 4)'
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0',
        description='USB port của LiDAR Camsense X2'
    )

    serial_port = LaunchConfiguration('serial_port')
    lidar_port  = LaunchConfiguration('lidar_port')

    # ── 1. STM32 Bridge (odom + cmd_vel) ─────────
    stm32_bridge = Node(
        package='restaurant_robot_bringup',
        executable='stm32_bridge',
        name='stm32_bridge',
        output='screen',
        parameters=[{
            'serial_port':    serial_port,
            'baud_rate':      115200,
            'base_frame':     'base_link',
            'odom_frame':     'odom',
            'publish_tf':     True,
            'max_linear_mms':    350.0,   # mm/s → 0.35 m/s
            'max_angular_rads':  1.2,     # rad/s
        }],
    )

    # ── 2. LiDAR Driver ───────────────────────────
    lidar_node = LifecycleNode(
        package='hclidar_driver_ros2',
        executable='hclidar_driver_ros2_node',
        name='hclidar_driver_ros2_node',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(lidar_dir, 'params', 'hclidar.yaml')],
        namespace='',
    )

    # ── 3. Static TF: base_link → laser_frame ─────
    # LiDAR gắn ở tâm robot, cách mặt sàn 2cm (theo thiết kế lidar_base.stl)
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar',
        arguments=[
            '--x',     '0.0',
            '--y',     '0.0',
            '--z',     '0.02',
            '--roll',  '0',
            '--pitch', '0',
            '--yaw',   '0',
            '--frame-id',       'base_link',
            '--child-frame-id', 'laser_frame',
        ],
    )

    # ── 4. SLAM Toolbox ───────────────────────────
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(bringup_dir, 'config', 'slam_toolbox_params.yaml'),
            {'use_sim_time': False},
        ],
    )

    return LaunchDescription([
        serial_port_arg,
        lidar_port_arg,
        LogInfo(msg='=== Restaurant Robot Bringup — Phase 1: MAPPING ==='),
        stm32_bridge,
        lidar_node,
        static_tf_lidar,
        slam_node,
    ])
