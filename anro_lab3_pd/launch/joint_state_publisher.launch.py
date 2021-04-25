import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'movable.xacro,urdf.xml'
    urdf = os.path.join( get_package_share_directory('anro_lab3_pd'), urdf_file_name)
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'),
        Node(
	    package='anro_lab3_pd',
	    executable='kdl_node',
	    name='KDL_DKIN',),
	Node(
	    package='anro_lab3_pd',
	    executable='nonkdl_node',
	    name='NONKDL_DKIN',),
    ])
