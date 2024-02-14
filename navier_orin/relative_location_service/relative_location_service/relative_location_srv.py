import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from math import atan2, degrees, sqrt
from tf_transformations import euler_from_quaternion
from custom_interfaces.srv import RelativeLocation  
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import math

class RelativeLocationServer(Node):

    def __init__(self):
        super().__init__('relative_location_srv')
        self.service_server = self.create_service(
            RelativeLocation,
            'get_relative_location',
            self.relative_location_callback)

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odometry',
            self.odom_callback,
            qos_profile_sensor_data)
        self.current_orientation = None
        self.current_trans_x = 0.0
        self.current_trans_y = 0.0

    def relative_location_callback(self, request, response):
        if self.current_orientation is None:
            self.get_logger().error('No current orientation available.')
            response.success = False
            response.message = 'No current orientation available.'
            return response

        target_x = request.x
        target_y = request.y

        # Draw vector from A to B

        delta_x = target_x - self.current_trans_x
        delta_y = target_y - self.current_trans_y

        # Find robot angle
        self_angle_rad = self.euler_from_quaternion(self.current_orientation)
        # Convert robot angle from radians to degrees
        self_angle_deg = math.degrees(self_angle_rad)
        # Calculate the angle between vector and x axis in radians
        angle_rad = math.atan2(delta_y, delta_x)

        # Convert the angle from radians to degrees.
        vector_angle = math.degrees(angle_rad)

        # Find the difference in degrees
        angle_to_rotate = vector_angle - self_angle_deg
        self.get_logger().info(f'Robot angle: {self_angle_deg} degrees, Vector angle: {vector_angle} ')


        # Normalize the result to the range [-180, 180]
        if angle_to_rotate > 180:
             angle_to_rotate -= 360
        elif angle_to_rotate < -180:
             angle_to_rotate += 360

        # Calculate the distance between two points
        distance = sqrt((target_x - self.current_trans_x)**2 + (target_y - self.current_trans_y)**2)

        self.get_logger().info(f'Calculated angle to rotate: {angle_to_rotate} degrees')
        self.get_logger().info(f'Calculated distance between points: {distance}')

        response.success = True
        response.message = 'Relative location and distance calculated.'
        response.angle_to_rotate = angle_to_rotate
        response.distance = distance
        return response

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        self.current_trans_x = msg.pose.pose.position.x
        self.current_trans_y = msg.pose.pose.position.y
        self.current_orientation = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Below should be replaced when porting for ROS2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return yaw

def main(args=None):
    rclpy.init(args=args)
    relative_location_server = RelativeLocationServer()
    rclpy.spin(relative_location_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
