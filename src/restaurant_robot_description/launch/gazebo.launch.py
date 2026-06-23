import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_robot = get_package_share_directory('restaurant_robot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    default_world = os.path.join(pkg_robot, 'worlds', 'obs.sdf')
    urdf_file     = os.path.join(pkg_robot, 'urdf', 'robot.urdf.xacro')
    rviz_config   = os.path.join(pkg_robot, 'rviz', 'robot_view.rviz')
    world = LaunchConfiguration('world')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 (true/false)')
    declare_world = DeclareLaunchArgument(
    'world',
    default_value=default_world,
    description='World file'
)

    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }]
    )

    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={
        'gz_args': [world, ' -r']
    }.items()
)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'restaurant_robot',
            '-x',     '-2.0',   # Dịch vào trong map, tránh out-of-bounds
            '-y',     '0.0',
            '-z',     '0.05',
            '-Y',     '0.0',    # yaw=0 (nhìn về phía +X)
        ],
        output='screen',
    )

    # Trì hoãn việc spawn robot 5 giây để Gazebo kịp khởi động và load world.
    # Không dùng RegisterEventHandler với target_action=gazebo vì gazebo là IncludeLaunchDescription.
    spawn_after_gazebo = TimerAction(
        period=5.0,
        actions=[spawn_robot]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # joint_states: plugin publish "joint_states" (không có /),
            # bridge cần remapping để ROS2 nhận đúng
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[('/joint_states', 'joint_states')],
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    # Thêm đường dẫn vào Gazebo để nó tìm thấy meshes/logo.png
    gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(get_package_prefix('restaurant_robot_description'), 'share')
    )

    # Gazebo Harmonic gpu_lidar bỏ qua gz:frame_id và dùng scoped name
    # (restaurant_robot/laser_frame/laser) làm frame_id trong LaserScan msg.
    # Static TF identity nối frame đó vào TF tree để RViz hiển thị được.
    laser_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'laser_frame',
            '--child-frame-id', 'restaurant_robot/laser_frame/laser',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Tương tự cho IMU sensor
    imu_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'imu_link',
            '--child-frame-id', 'restaurant_robot/imu_link/imu_sensor',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        gz_resource_path,
        declare_rviz,
        declare_world,
        robot_state_publisher,
        gazebo,
        bridge,
        spawn_after_gazebo,  # <-- không phải spawn_robot trực tiếp
        laser_frame_bridge,
        imu_frame_bridge,
        rviz2,
    ])