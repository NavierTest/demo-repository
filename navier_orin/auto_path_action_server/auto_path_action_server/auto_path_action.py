import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from custom_interfaces.action import AutoPath
from custom_interfaces.srv import RelativeLocation
from geometry_msgs.msg import Twist
import math

class RelativeLocationServer(Node):
    def __init__(self):
        super().__init__('relative_location_server')
        self.server = ActionServer(self, AutoPath, 'autopath_as', self.execute_callback)
        self.relative_location_client = self.create_client(RelativeLocation, 'get_relative_location')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_force', 10)
        self.twist = Twist()
        self.stationary_turn_threshold = 55  # Turn treshold while stationary
        self.moving_turn_threshold = 5       # Turn treshold while moving towards target

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal request')
        self.feedback_msg = AutoPath.Feedback()
        
        x = goal_handle.request.x
        y = goal_handle.request.y

        try:
            while rclpy.ok():
                response = await self.call_relative_location_service(x, y)
                if response:
                    self.turn_degrees = response.angle_to_rotate
                    self.distance = response.distance
                    self.feedback_msg.degrees = self.turn_degrees
                    self.feedback_msg.distance = self.distance
                    goal_handle.publish_feedback(self.feedback_msg)

                    # Check wich way to rotate

                    if self.turn_degrees > 0:
                        turnDirection = 1
                    elif self.turn_degrees <= 0:
                        turnDirection = -1

                    # Calculate movement speed

                    if self.distance < 0.2:
                        move_speed_multiplier = 0.2

                    elif self.distance < 0.5:
                        move_speed_multiplier = 0.5

                    elif self.distance < 1.0:
                        move_speed_multiplier = 0.8
                    
                    else:
                        move_speed_multiplier = 1.0
                

                    # Aimed at target

                    if abs(self.turn_degrees) <= self.stationary_turn_threshold:
                        self.twist.angular.z = 0.2 * turnDirection  
                        self.twist.linear.x = 0.5  * move_speed_multiplier
                        self.cmd_vel_publisher.publish(self.twist)
                    
                    # Not aimed at target
                    else:
                        
                        self.twist.angular.z = 0.7 * turnDirection  
                        self.twist.linear.x = 0.0
                        self.cmd_vel_publisher.publish(self.twist)

                    # Arrived at target
                    if self.distance < 0.1:
                        
                        self.twist.angular.z = 0.0
                        self.twist.linear.x = 0.0
                        self.cmd_vel_publisher.publish(self.twist)
                        self.get_logger().info('Goal executed successfully')
                        goal_handle.succeed()
                        result = AutoPath.Result()
                        result.success = True
                        return result
        except Exception as e:
            self.get_logger().error(f"Failed to execute action: {str(e)}")
            goal_handle.abort()
            result = AutoPath.Result()
            return result

    async def call_relative_location_service(self, x, y):
        srv_req = RelativeLocation.Request()
        srv_req.x = x
        srv_req.y = y

        try:
            response = await self.relative_location_client.call_async(srv_req)
            return response
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    relative_location_server = RelativeLocationServer()
    rclpy.spin(relative_location_server)
    relative_location_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
