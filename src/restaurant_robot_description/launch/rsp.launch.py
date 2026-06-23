"""
rsp.launch.py
Package: restaurant_robot_description
Description: Launch file để khởi động Robot State Publisher.
             Load file URDF/Xacro và publish robot description lên /robot_description.
             Joint State Publisher được kích hoạt để publish trạng thái joint.

Nodes khởi động:
  1. robot_state_publisher : Publish TF từ URDF và /robot_description
  2. joint_state_publisher  : Publish joint states (dùng khi chạy không có Gazebo)

Arguments:
  use_sim_time (bool, default: false) : Dùng simulation time từ Gazebo
  
ROS Version: ROS 2 Jazzy Jalisco
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ----------------------------------------------------------------
    # Đường dẫn đến package
    # ----------------------------------------------------------------
    pkg_share = get_package_share_directory('restaurant_robot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    # ----------------------------------------------------------------
    # Declare launch arguments
    # ----------------------------------------------------------------
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Dùng simulation time (true khi chạy với Gazebo)'
    )

    # ----------------------------------------------------------------
    # Xử lý URDF bằng xacro
    # Command(['xacro ', urdf_file]) sẽ chạy lệnh xacro tại runtime
    # ParameterValue(..., value_type=str) đảm bảo kiểu dữ liệu đúng
    # ----------------------------------------------------------------
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ----------------------------------------------------------------
    # Node 1: Robot State Publisher
    # Đọc URDF và publish:
    #   - /robot_description (std_msgs/String)
    #   - /tf, /tf_static (geometry_msgs/TransformStamped)
    # ----------------------------------------------------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }]
    )

    # ----------------------------------------------------------------
    # Node 2: Joint State Publisher
    # Publish joint states cho các joint non-fixed (bánh xe).
    # Chỉ cần khi KHÔNG dùng Gazebo (Gazebo tự publish joint states
    # thông qua plugin diff_drive).
    # Dùng joint_state_publisher_gui để có giao diện slider nếu muốn.
    # ----------------------------------------------------------------
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # ----------------------------------------------------------------
    # Tạo LaunchDescription
    # ----------------------------------------------------------------
    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher_node,
        #joint_state_publisher_node,
    ])
