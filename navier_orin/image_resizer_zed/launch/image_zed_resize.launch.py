from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
            name='image_proc_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Image Format Converter Node
                ComposableNode(
                    package='isaac_ros_image_proc',
                    plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                    name='image_format_converter_node',
                    remappings=[
                        ('image_raw', '/zed/zed_node/rgb/image_rect_color'),
                        ('camera_info', '/zed/zed_node/rgb/camera_info'),
                        ('image', '/zed/zed_node/rgb/image_rect_color_converted'),
                    ],
                    parameters=[
                        {'encoding_desired': 'bgr8'},
                    ],
                ),
                # Resize Node
                ComposableNode(
                    package='isaac_ros_image_proc',
                    plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                    name='resize_node',
                    parameters=[
                        {'output_width': 1920},
                        {'output_height': 1088},
                        {'num_blocks': 40},
                        {'keep_aspect_ratio': False},
                        {'encoding_desired': 'bgr8'}
                    ],
                    remappings=[
                        ('/image', '/zed/zed_node/rgb/image_rect_color_converted'),
                        ('/camera_info', '/zed/zed_node/rgb/camera_info'),
                    ],
                )
            ],
            output='screen',
    )

    return LaunchDescription([container])