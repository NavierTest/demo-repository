import serial
import Jetson.GPIO as GPIO
import time

import rclpy
from rclpy.node import Node
import numpy as np
import sys
from geometry_msgs.msg import Twist

class RcInnput(Node):

    def __init__(self, serial_port='/dev/ttyUSB1', baud_rate=9600, channel_count=14, pwm_pin=18):
        super().__init__('rc_input_node')
        self.serial_port = serial_port
        self.channel_count = channel_count
        self.baud_rate = baud_rate
        self.pwm_pin = pwm_pin
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_force', 10)

        
        # Initialize serial port
        self.ser = serial.Serial(self.serial_port, self.baud_rate)

        input("Press Enter to proceed")
    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()

    def map_value(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def read_serial_input(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        #line = self.ser.readline().decode('utf-8').strip()
        raw_channels = line.split(',')
        self.channels = []
        for i in range(min(self.channel_count, len(raw_channels))):
            try:
                self.channels.append(int(raw_channels[i]))
            except ValueError:
                print(f"Can't read channel {i}")

    def scale_value(self, ch):
        value = self.channels[ch]
        value_scaled = (((float(value) - 588) / 510) - 1)

        if value_scaled > 1:
            value_scaled = 1.0
        if value_scaled < -1:
            value_scaled = -1.0
        if abs(value_scaled) < 0.02:
            value_scaled = 0.0

        return value_scaled
    def publish_message(self):
        self.read_serial_input()
        
        msg = Twist()

        # Setting linear and angular velocity components from channels 5 and 7
        msg.linear.x = self.scale_value(5)
        msg.angular.z = self.scale_value(7)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


    def main_loop(self):
        self.timer = self.create_timer(0.05, self.publish_message)  # Adjust the timer value as needed
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.cleanup()
            print("Exiting...")


def main(args=None):
    rclpy.init(args=args)
    rc_input = RcInnput()
    rc_input.main_loop()
    rclpy.shutdown()
