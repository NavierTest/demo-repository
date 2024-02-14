import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
import utm
import numpy as np

class LocalizationNode(Node):
    def __init__(self):
        # Initialize the Node with the name 'localization_node'
        super().__init__('localization_node')

        # Declare a parameter for publishing frequency with a default value of 200 Hz
        self.declare_parameter('frequency', 200)
        # Retrieve the value of the 'frequency' parameter
        self.frequency = self.get_parameter('frequency').value

        # Create publishers and subscribers for the necessary topics
        self.odom_pub = self.create_publisher(Odometry, '/odometry', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/gnss', self.gps_callback, 10)

        # Initialize some variables to hold the initial UTM coordinates and orientation
        self.initial_utm = None
        self.orientation = None

        # Initialize an empty Odometry message
        self.odom_msg = self.init_odom_msg()

        # Create a timer to call the publish_odometry method at the specified frequency
        self.timer = self.create_timer(1.0 / self.frequency, self.publish_odometry)

    def init_odom_msg(self):
        # Initialize an empty Odometry message with the frame IDs set
        odom_msg = Odometry()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        return odom_msg

    def imu_callback(self, msg):
        # Update the orientation and angular velocity in the odometry message when new IMU data is received
        self.orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        # self.odom_msg.twist.twist.angular = msg.angular_velocity
        self.odom_msg.pose.pose.orientation = msg.orientation

    def gps_callback(self, msg):
        # Convert the GPS coordinates to UTM coordinates when new GPS data is received
        current_utm = utm.from_latlon(msg.latitude, msg.longitude)
        # If this is the first GPS message, save the UTM coordinates as the initial position
        if self.initial_utm is None:
            self.initial_utm = np.array([current_utm[0], current_utm[1]])
        # Calculate the displacement from the initial position and update the position in the odometry message
        displacement = np.array([current_utm[0], current_utm[1]]) - self.initial_utm
        self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y = displacement

    def publish_odometry(self):
        # Publish the odometry message and the transform if the orientation has been received
        if self.orientation is not None:
            # Update the timestamp in the odometry message
            self.odom_msg.header.stamp = self.get_clock().now().to_msg()
            # Publish the odometry message
            self.odom_pub.publish(self.odom_msg)


def main(args=None):
    # Initialize the ROS2 environment
    rclpy.init(args=args)
    # Create an instance of the LocalizationNode class
    localization_node = LocalizationNode()
    # Spin the node to process callbacks and keep the node running
    rclpy.spin(localization_node)
    # Destroy the node and shutdown the ROS2 environment when the node is stopped
    localization_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # Call the main function if this script is executed
    main()
