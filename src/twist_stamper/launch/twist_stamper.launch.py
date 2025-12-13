from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='twist_stamper',
            executable='twist_stamper',
            name='twist_stamper',
            output='screen',
            parameters=[
                {'twist_topic_in': '/cmd_vel'},
                {'twiststamped_topic_out': '/diff_cont/cmd_vel'},
            ]
        )
    ])
