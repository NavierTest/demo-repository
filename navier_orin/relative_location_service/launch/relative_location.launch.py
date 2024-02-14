from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='relative_location_service',
            executable='relative_location_srv',
            output='screen'),
    ])