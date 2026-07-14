#!/usr/bin/env python3
"""
navigation.launch.py — Phase 2: NAVIGATION
Chạy trên Raspberry Pi 4 sau khi đã có map từ Phase 1.

Khởi động:
  1. STM32 bridge (odom + cmd_vel)
  2. LiDAR driver
  3. Static TF: base_link → laser_frame
  4. Map server (load map đã lưu)
  5. AMCL (localization)
  6. Nav2 stack (controller, planner, bt_navigator...)

Lệnh chạy:
  source ~/dev_ws/install/setup.bash
  ros2 launch restaurant_robot_bringup navigation.launch.py \\
      map:=$HOME/map/restaurant_map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('restaurant_robot_bringup')
    lidar_dir   = get_package_share_directory('hclidar_driver_ros2')

    nav2_params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    # ── Launch arguments ──────────────────────────
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            os.path.expanduser('~'), 'map', 'restaurant_map.yaml'
        ),
        description='Path tới file map.yaml đã lưu',
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyAMA0',
        description='UART port kết nối STM32',
    )

    map_file    = LaunchConfiguration('map')
    serial_port = LaunchConfiguration('serial_port')

    # ── 1. STM32 Bridge ───────────────────────────
    stm32_bridge = Node(
        package='restaurant_robot_bringup',
        executable='stm32_bridge',
        name='stm32_bridge',
        output='screen',
        parameters=[{
            'serial_port':       serial_port,
            'baud_rate':         115200,
            'base_frame':        'base_link',
            'odom_frame':        'odom',
            'publish_tf':        True,
            'max_linear_mms':    350.0,
            'max_angular_rads':  1.2,
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

    # ── 3. Static TF ──────────────────────────────
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.02',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser_frame',
        ],
    )

    # ── 4. Map Server ─────────────────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'yaml_filename': map_file},
        ],
    )

    # ── 5. AMCL ───────────────────────────────────
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file],
    )

    # ── 6. Nav2 Nodes ─────────────────────────────
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )

    # ── 7. Lifecycle Managers ─────────────────────
    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[nav2_params_file],
    )

    lifecycle_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params_file],
    )

    return LaunchDescription([
        map_arg,
        serial_port_arg,
        LogInfo(msg='=== Restaurant Robot Bringup — Phase 2: NAVIGATION ==='),
        stm32_bridge,
        lidar_node,
        static_tf_lidar,
        map_server,
        amcl,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_localization,
        lifecycle_navigation,
    ])
