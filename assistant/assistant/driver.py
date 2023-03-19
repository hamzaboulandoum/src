import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import serial
import numpy as np
import math
import time

#constants
WHEEL_RADIUS = 0.05
DISTANCE_FROM_CENTER = 0.18
SYSTEM_MATRIX = 1/WHEEL_RADIUS*np.array([[1, 0, - DISTANCE_FROM_CENTER],[-1/2, -np.sqrt(3)/2, - DISTANCE_FROM_CENTER],[-1/2, np.sqrt(3)/2, - DISTANCE_FROM_CENTER]])

class Driver(Node):
    def __init__(self):
        super().__init__('velocity_to_angle_node')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.callback, 10)
        self.serial = serial.Serial('/dev/serial/by-path/pci-0000:00:14.0-usb-0:3:1.0', 9600) # change it for raspberry device when available
        self.publisher = self.create_publisher(Odometry, 'odom', 10)
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_t = time.time()

    def callback(self, msg):
        # Convert velocity commands to wheel speeds for a three-wheeled omnidirectional robot
        linear_velocity_x = msg.linear.x
        linear_velocity_y = msg.linear.y

        if linear_velocity_x == 0 and linear_velocity_y == 0:
            vx = 0
            vy = 0
        else :
            vx = linear_velocity_x/math.sqrt(linear_velocity_x**2 + linear_velocity_y**2)
            vy = linear_velocity_y/math.sqrt(linear_velocity_x**2 + linear_velocity_y**2)
        
        data = str(vx) + " " + str(vy)        
        self.serial.write(data.encode()) # sending data to arduino 

        # encoders
        #a = len(encoder_data)
        encoder_data = self.serial.readline().decode().strip().split(', ')
        while len(encoder_data) < 3 :
            encoder_data = self.serial.readline().decode().strip().split(', ')

        self.get_logger().info(str(encoder_data))

        # Parse encoder data and calculate odometry values for a three-wheeled omnidirectional robot
        u_1, u_2, u_3 = map(float, encoder_data)
        U = np.array([u_1, u_2, u_3])
        velocity = np.dot(np.linalg.inv(SYSTEM_MATRIX),U)
        v_x = velocity[0]
        v_y = velocity[1]

        
        t = time.time()

        x = self.prev_x + (t-self.prev_t)*v_x
        y = self.prev_y + (t-self.prev_t)*v_y

        self.prev_x = x
        self.prev_y = y
        self.prev_t = t
        
        odom_msg = Odometry()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.twist.twist.linear.x = v_x
        odom_msg.twist.twist.linear.y = v_y
      
        self.publisher.publish(odom_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()




    