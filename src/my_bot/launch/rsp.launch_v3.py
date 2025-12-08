import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable # <--- Thêm FindExecutable
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

def generate_launch_description():

    package_name = 'my_bot'

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')

    # Paths
    pkg_path = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    # Generate robot_description using xacro command
    # --- SỬA LỖI TẠI ĐÂY ---
    # Dùng FindExecutable và thêm khoảng trắng ' ' để tách lệnh và đường dẫn
    robot_description_content = Command([
        FindExecutable(name='xacro'), 
        ' ', 
        urdf_path
    ])
    
    robot_description = ParameterValue(robot_description_content, value_type=str)
    # -----------------------

    # Nodes
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # GUI (optional)
    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(use_gui),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_gui', default_value='true'),
        rsp_node,
        joint_state_gui
    ])