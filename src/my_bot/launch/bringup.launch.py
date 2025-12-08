import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


def generate_launch_description():

    # --- Simulation time ---
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Load URDF / Xacro ---
    pkg_path = get_package_share_directory('my_bot')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    params = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time
    }

    # --- Node: robot_state_publisher ---
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # --- Node: joint_state_publisher_gui ---
    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # --- Node: Lidar driver (sllidar_node) ---
    node_sllidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 115200,   # <-- MUST BE INTEGER
            'frame_id': 'linklidar',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Express'
        }]
)


    # --- Node: RViz2 ---
    # Nếu bạn có file rviz config thì để vào:
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'my_bot.rviz')

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # ----------------- Launch -----------------
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        node_robot_state_publisher,
        node_joint_state_publisher,
        node_sllidar,          # 🔥 Lidar driver
        node_rviz,             # 🔥 Mở RViz tự động
    ])
