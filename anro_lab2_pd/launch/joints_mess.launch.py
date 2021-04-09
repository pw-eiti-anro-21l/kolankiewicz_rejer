from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
	Node(
	    prefix='gnome-terminal --',
	    package='anro_lab2_pd',
	    executable='teleop_talker',
	    name='state_publisher_teleop',
	    output='screen',
	    emulate_tty=True,
	    parameters=[
	        {'up_key': 'q'},
	        {'down_key': 'w'},
	        {'first_left_key': 'e'},
	        {'first_right_key': 'r'},
	        {'second_left_key': 't'},
	        {'second_right_key': 'u'},
	    ]
	),
	])
    """
        Node(
            package='anro_lab2_pd',
            executable='state_publisher',
            name='state_publisher',
        ),
    ])"""
