from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            prefix='gnome-terminal --',
            package='anro_lab1_pd',
            executable='teleop_talker',
            name='custom_teleop',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'forward_key': 'o'},
                {'backward_key': 'l'},
                {'left_key': 'k'},
                {'right_key': 'p'}
            ]
        )
    ])
