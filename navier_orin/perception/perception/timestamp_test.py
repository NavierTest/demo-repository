from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import rclpy
from rclpy.node import Node

class Timestamps(Node):
    def __init__(self):
        super().__init__('timestamps_sync_test')

        # Create subscriptions to CameraInfo and PointCloud2 topics
        self.create_subscription(CameraInfo, '/zed/zed_node/rgb/camera_info', self.camera_info_callback, 10)
        self.create_subscription(PointCloud2, '/velodyne_points', self.lidar_callback, 10)

    def camera_info_callback(self, msg):
        # Log the reception of CameraInfo message and its timestamp
        self.get_logger().info(f'Received Camera Info message with timestamp: {msg.header.stamp}')

    def lidar_callback(self, msg):
        # Log the reception of PointCloud2 message and its timestamp
        self.get_logger().info(f'Received LiDAR message with timestamp: {msg.header.stamp}')

def main(args=None):
    rclpy.init(args=args)
    timestamp_node = Timestamps()
    rclpy.spin(timestamp_node)
    timestamp_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
