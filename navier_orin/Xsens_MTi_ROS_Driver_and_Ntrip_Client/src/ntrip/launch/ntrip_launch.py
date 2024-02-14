import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='ntrip',
            executable='ntrip',
            name='ntrip_client',
            output='screen',
            parameters=[
                {'ip': '159.162.103.14'},  # Change to the IP address of Your NTRIP service
                {'port': 2101},  # Change to your port number
                {'user': 'navierusn'},  # Change to your username
                {'passwd': 'nav123'},  # Change to your password
                {'mountpoint': 'CPOSRTCM32'},  # Change to your mountpoint
                {'report_interval': 10} # the report interval to the NTRIP Caster, default is 1 sec
            ]
        ),
    ])
    
