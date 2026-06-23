"""
nav2.launch.py
Package: restaurant_robot_navigation
Description: Khởi động Nav2 stack đầy đủ cho robot nhà hàng (differential drive).
             Bao gồm: map_server, AMCL, controller, planner, behavior, bt_navigator.

Arguments:
  map          : Đường dẫn đến file map YAML (default: restaurant_map.yaml)
  use_sim_time : Dùng simulation time (default: true)
  params_file  : Đường dẫn đến nav2_params.yaml
  autostart    : Tự động activate lifecycle nodes (default: true)

Usage (riêng lẻ, sau khi đã chạy gazebo.launch.py):
  ros2 launch restaurant_robot_navigation nav2.launch.py

Usage (kết hợp với Gazebo):
  ros2 launch restaurant_robot_navigation nav2_sim.launch.py

ROS Version: ROS 2 Jazzy Jalisco
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_robot = get_package_share_directory('restaurant_robot_navigation')

    # ----------------------------------------------------------------
    # Default paths
    # ----------------------------------------------------------------
    default_map    = os.path.join(pkg_robot, 'maps', 'restaurant_map.yaml')
    default_params = os.path.join(pkg_robot, 'config', 'nav2_params.yaml')

    # ----------------------------------------------------------------
    # Launch arguments
    # ----------------------------------------------------------------
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map YAML file'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to nav2_params.yaml'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Auto-activate nav2 lifecycle nodes on startup'
    )

    # ----------------------------------------------------------------
    # Resolve launch configurations
    # ----------------------------------------------------------------
    map_yaml     = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('params_file')
    autostart    = LaunchConfiguration('autostart')

    # ----------------------------------------------------------------
    # Node 1: Map Server
    # Serve bản đồ đã tạo trước (localization mode)
    # ----------------------------------------------------------------
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'yaml_filename': map_yaml,
             'use_sim_time': use_sim_time}
        ],
    )

    # ----------------------------------------------------------------
    # Node 2: AMCL — Adaptive Monte Carlo Localization
    # Dùng bản đồ + LiDAR scan để ước lượng pose robot
    # ----------------------------------------------------------------
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    # ----------------------------------------------------------------
    # Node 3: Lifecycle Manager — Localization
    # Quản lý vòng đời (configure → activate) cho map_server + amcl
    # ----------------------------------------------------------------
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    # ----------------------------------------------------------------
    # Node 4: Controller Server — DWB Local Planner
    # Tính velocity command để bám theo path
    # ----------------------------------------------------------------
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[('cmd_vel', 'cmd_vel')],
    )

    # ----------------------------------------------------------------
    # Node 5: Planner Server — NavFn Global Planner
    # Tạo đường đi tổng thể từ vị trí hiện tại đến goal
    # ----------------------------------------------------------------
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    # ----------------------------------------------------------------
    # Node 6: Behavior Server — Recovery behaviors
    # Cung cấp hành vi phục hồi: spin, backup, wait
    # ----------------------------------------------------------------
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[('cmd_vel', 'cmd_vel')],
    )

    # ----------------------------------------------------------------
    # Node 7: BT Navigator — Điều phối navigation qua Behavior Tree
    # Nhận goal từ action client, gọi planner + controller
    # ----------------------------------------------------------------
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    # ----------------------------------------------------------------
    # Node 8: Lifecycle Manager — Navigation
    # Quản lý vòng đời cho toàn bộ navigation stack
    # ----------------------------------------------------------------
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ],
        }],
    )

    return LaunchDescription([
        # Arguments
        declare_map,
        declare_use_sim_time,
        declare_params,
        declare_autostart,

        # Localization stack
        map_server,
        amcl,
        lifecycle_manager_localization,

        # Navigation stack
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_navigation,
    ])
