from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    zed_wrapper_launch_file = PathJoinSubstitution([
        get_package_share_directory('zed_wrapper'),
        'launch',
        'zed_camera.launch.py'
    ])

    velodyne_driver_launch_file = PathJoinSubstitution([
        get_package_share_directory('velodyne_driver'),
        'launch',
        'velodyne_driver_node-VLP16-launch.py'
    ])

    velodyne_pointcloud_launch_file = PathJoinSubstitution([
        get_package_share_directory('velodyne_pointcloud'),
        'launch',
        'velodyne_transform_node-VLP16-launch.py'
    ])

    return LaunchDescription([
        # Include ZED Camera launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_wrapper_launch_file),
            launch_arguments={'camera_model': 'zedx', 'camera_name': 'zed'}.items()
        ),
        
        # Include Velodyne driver launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(velodyne_driver_launch_file)
        ),

        # Include Velodyne pointcloud launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(velodyne_pointcloud_launch_file)
        ),

        # Static Transform Publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0.08', '0.06', '0.01', '0.5', '-0.5', '0.5', '0.5', 'zed_left_camera_optical_frame', 'velodyne']
            # 0.06000000000000001 0.016000000000000007 0.009999999999999978
        )
        


    ])
