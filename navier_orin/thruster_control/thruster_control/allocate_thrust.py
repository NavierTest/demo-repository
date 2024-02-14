import serial
import Jetson.GPIO as GPIO
import time
from ament_index_python.packages import get_package_share_directory
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
from pandas import read_excel
import time
from std_msgs.msg import String

class AllocataThrust(Node):
    def __init__(self):
        super().__init__("rc_input_node")
        GPIO.setwarnings(False) # no warings if this line is here

        #define thruster limits
        self.main_thruster_max = 25 #bi-directional maximum
        self.l_m = 1 #distance from centre of gravity to main thrusters (y-direction, m)
        forward_force_limit_factor = 1  #1 #0.5 #0.1  #0.5 #1
        self.max_surge_force = 2*self.main_thruster_max*forward_force_limit_factor #maximum surge force achievable by main thrusters
        self.max_yaw_moment = 2*self.main_thruster_max*self.l_m #maximum yaw moment achievable by all thrusters

        # Load thruster data from excel file and parse the necessary data
        #PWM deadzone: [1464, 1536]
        g = 9.08665 #m/s^2
        package_share_directory = get_package_share_directory('thruster_control')
        excel_file_path = os.path.join(package_share_directory, 't200_data', 'thruster_data.xlsx')
        self.thruster_data_file = read_excel(excel_file_path, '12 V', engine='openpyxl')  # read pwm-force data for 12V t200 thrusters

        self.thruster_matrix = np.array(self.thruster_data_file)
        # Preprocess thruster data
        self.thruster_dataArray = np.array(self.thruster_matrix[:, 5]) #Element [91:109] = 0
        self.p_thruster_dataArray = (self.thruster_dataArray[110:201]*g) #from kg to N
        self.n_thruster_dataArray = (self.thruster_dataArray[:91]*(-1)*g)
        # Preprocess PWM data
        self.pwm_dataArray   = np.array(self.thruster_matrix[:, 0])
        self.p_pwm_dataArray = np.array(self.pwm_dataArray[110:201])
        self.n_pwm_dataArray = (self.pwm_dataArray[:91])
        self.neutral_pwm = 1500/1000

        # Initialize PWM channels for thruster control
        # Setup Jetson Nano GPIO
        self.left_pwm_channel = 18   
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.left_pwm_channel, GPIO.OUT)
        self.left_pwm_channel = GPIO.PWM(self.left_pwm_channel, 50)
        self.left_pwm_channel.start(7.5)
        self.right_pwm_channel = 15 
        GPIO.setup(self.right_pwm_channel, GPIO.OUT)
        self.right_pwm_channel = GPIO.PWM(self.right_pwm_channel, 50)
        self.right_pwm_channel.start(7.5)
        time.sleep(1)
        self.get_logger().info('Initializing T200 PWM channels')
        

        # Create a subscriber to the command topic to receive force and torque commands
        self.cmd_sub = self.create_subscription(Twist, '/cmd_force', self.force_command_cb, 10)

    def cleanup(self):
        # Cleanup function to stop PWM signals and cleanup GPIO settings
        self.left_pwm_channel.stop()
        self.right_pwm_channel.stop()
        GPIO.cleanup()
        

    def allocate_thrust(self, F_desired, M_desired):
        # This function takes in desired force and moment to calculate the thrust needed from each thruster
        if abs(M_desired) > self.max_yaw_moment: #if desired moment is greater than max
            left_thrust = self.main_thruster_max*np.sign(M_desired) #set max differential thrust
            right_thrust = -left_thrust
            
            return left_thrust, right_thrust
        else: #if desired moment is achievable
            left_thrust = M_desired/(2*self.l_m)
            right_thrust = -left_thrust

        #attempt to achieve desired surge thrust
        remaining_main_thrust = self.main_thruster_max-abs(right_thrust) #remaining thrust available after moment is allocated
        right_thrust += np.min([remaining_main_thrust, abs(F_desired)/2])*np.sign(F_desired)
        left_thrust  += np.min([remaining_main_thrust, abs(F_desired)/2])*np.sign(F_desired)
        return left_thrust, right_thrust



    def map_thrust_to_PWM(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def force_command_cb(self, msg):

        left_thrust, right_thrust = 0, 0 
        F_desired = msg.linear.x*self.max_surge_force #scale from [-1, 1] to force in newton
        M_desired = msg.angular.z*self.max_yaw_moment
        left_thrust, right_thrust = self.allocate_thrust(F_desired, M_desired)

        #self.get_logger().info('commanding thrust: [' + str(left_thrust) + ', ' + str(right_thrust) + ']')

        duty_cycle_left_thruster = self.map_thrust_to_PWM(left_thrust, -25, 25, 5.5, 9.5)
        duty_cycle_right_thruster = self.map_thrust_to_PWM(right_thrust, -25, 25, 5.5, 9.5)
        
        self.left_pwm_channel.ChangeDutyCycle(duty_cycle_left_thruster)
        self.right_pwm_channel.ChangeDutyCycle(duty_cycle_right_thruster)

    def main_loop(self):
        while rclpy.ok():
            try:
                rclpy.spin(self)
            except KeyboardInterrupt:
                self.cleanup()
                print("Exiting...")



def main(args=None):
    rclpy.init(args=args)
    
    allocata_thrust = AllocataThrust()
    
    try:
        allocata_thrust.main_loop()
    except Exception as e:
        allocata_thrust.get_logger().error('Caught exception: ' + str(e))
    finally:
        allocata_thrust.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

