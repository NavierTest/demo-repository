from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='odometry_publisher',
			executable='odometry_publisher',
			output='screen',
		),
	])
