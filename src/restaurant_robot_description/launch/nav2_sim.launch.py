"""
nav2_sim.launch.py
Package: restaurant_robot_description
Description: Launch tổng hợp — khởi động cả Gazebo + Nav2 cùng một lúc.
             Chạy gazebo.launch.py trước, sau đó delay 8 giây rồi khởi động
             nav2.launch.py để đảm bảo Gazebo và robot đã sẵn sàng.

Arguments:
  world        : Đường dẫn đến SDF world file (default: obs.sdf)
  map          : Đường dẫn đến map YAML file (default: restaurant_map.yaml)
  params_file  : Đường dẫn đến nav2_params.yaml
  rviz         : Có mở RViz2 không (default: true)
  use_sim_time : Dùng simulation time (default: true)

Usage:
  ros2 launch restaurant_robot_description nav2_sim.launch.py
  ros2 launch restaurant_robot_description nav2_sim.launch.py rviz:=false

ROS Version: ROS 2 Jazzy Jalisco
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_robot = get_package_share_directory('restaurant_robot_description')

    # ----------------------------------------------------------------
    # Default paths
    # ----------------------------------------------------------------
    default_world  = os.path.join(pkg_robot, 'worlds', 'obs.sdf')
    default_map    = os.path.join(pkg_robot, 'maps', 'restaurant_map.yaml')
    default_params = os.path.join(pkg_robot, 'config', 'nav2_params.yaml')

    # ----------------------------------------------------------------
    # Launch arguments
    # ----------------------------------------------------------------
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to Gazebo world SDF file'
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map YAML file'
    )
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to nav2_params.yaml'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 with Nav2 config (true/false)'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation time'
    )

    # ----------------------------------------------------------------
    # Sub-launch 1: Gazebo simulation
    # ----------------------------------------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    # ----------------------------------------------------------------
    # Sub-launch 2: Nav2 stack
    # Delay 10 giây để Gazebo kịp khởi động và spawn robot xong
    # (gazebo.launch.py đã delay 5s cho spawn, cộng thêm 5s buffer)
    # ----------------------------------------------------------------
    nav2_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_robot, 'launch', 'nav2.launch.py')
                ),
                launch_arguments={
                    'map':          LaunchConfiguration('map'),
                    'params_file':  LaunchConfiguration('params_file'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items(),
            )
        ]
    )

    return LaunchDescription([
        # Arguments
        declare_world,
        declare_map,
        declare_params,
        declare_rviz,
        declare_use_sim_time,

        # Launch sequence
        gazebo_launch,
        nav2_launch,
    ])
