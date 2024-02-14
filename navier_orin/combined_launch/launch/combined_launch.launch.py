from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='ntrip',
			executable='ntrip',
			output='screen',
                        name='ntrip_client',
               	        parameters=[
                		{'ip': '159.162.103.14'},  # Change to the IP address of Your NTRIP service
                		{'port': 2101},  # Change to your port number
                		{'user': 'navierusn'},  # Change to your username
                		{'passwd': 'nav123'},  # Change to your password
                		{'mountpoint': 'CPOSRTCM32'},  # Change to your mountpoint
                		{'report_interval': 10} # the report interval to the NTRIP Caster, default is 1 sec
            		]
        	),
		Node(
			package='xsens_mti_ros2_driver',
			executable='xsens_mti_node',
			output='screen',
		),
		Node(
			package='odometry_publisher',
			executable='odometry_publisher',
			output='screen',
		),
	])
