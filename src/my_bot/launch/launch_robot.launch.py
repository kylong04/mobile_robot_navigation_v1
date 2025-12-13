import os
import subprocess
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = get_package_share_directory('my_bot')
    urdf_path = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    processed_urdf = subprocess.run(
        ['xacro', urdf_path], 
        capture_output=True, 
        text=True, 
        check=True
    ).stdout

    robot_description = ParameterValue(processed_urdf, value_type=str)

    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(pkg_path, 'launch', 'bringup.launch.py')]
            ),
            launch_arguments={'use_sim_time': 'true',}.items()
        )

    controller_params_file = os.path.join(pkg_path, 'config', 'my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description},
            controller_params_file
        ],
        output="screen",
        arguments=['--ros-args', '--log-level', 'debug']
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["diff_cont", "-c", "/controller_manager", "--controller-manager-timeout", "100"],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_broad", "-c", "/controller_manager", "--controller-manager-timeout", "100"],
    )

    twist_stamper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(
                get_package_share_directory('twist_stamper'),
                'launch',
                'twist_stamper.launch.py'
            )]
        )
    )

    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom',
        # x  y  z  yaw pitch roll  parent  child
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        output='screen',
    )


    delayed_controller_manager = TimerAction(period=1.0, actions=[controller_manager])

    delayed_diff_drive_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner]
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner]
        )
    )

    return LaunchDescription([
        static_tf_world_odom,
        rsp,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        twist_stamper_launch,
    ])
