import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import numpy as np


import math

#constants
WHEEL_RADIUS = 0.05
DISTANCE_FROM_CENTER = 0.19

SYSTEM_MATRIX = 1/WHEEL_RADIUS*np.array([[1, 0, - DISTANCE_FROM_CENTER],[-1/2, -np.sqrt(3)/2, - DISTANCE_FROM_CENTER],[-1/2, np.sqrt(3)/2, - DISTANCE_FROM_CENTER]])

class Driver(Node):
    def __init__(self):
        super().__init__('driver')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.callback,
            10)
        
        self.declare_parameter('serial_port', value="/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0")
        self.serial_port = self.get_parameter('serial_port').value

        self.declare_parameter('baud_rate', value=9600)
        self.baud_rate = self.get_parameter('baud_rate').value

        self.serial = serial.Serial(self.serial_port, self.baud_rate)

        

    def callback(self, msg):
        # Convert velocity commands to wheel speeds for a three-wheeled omnidirectional robot
        linear_velocity_x = msg.linear.x
        linear_velocity_y = msg.linear.y
        w = msg.angular.z
        

        if linear_velocity_x == 0 and linear_velocity_y == 0:
            vx = 0
            vy = 0
        else :
            vx = linear_velocity_x/math.sqrt(linear_velocity_x**2 + linear_velocity_y**2)
            vy = linear_velocity_y/math.sqrt(linear_velocity_x**2 + linear_velocity_y**2)
        
        vx = vx * 0.15
        vy = vy * 0.15

        
        data = str(vx) + " " + str(vy) + " " + str(w)
        self.get_logger().info(str(data))
        self.serial.write(data.encode()) # sending data to arduino




def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    rclpy.spin(node)
    node.serial.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
