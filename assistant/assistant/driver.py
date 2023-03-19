import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import serial
import numpy as np
import math

#constants
WHEEL_RADIUS = 0.05
DISTANCE_FROM_CENTER = 0.18
SYSTEM_MATRIX = 1/WHEEL_RADIUS*np.array([[1, 0, - DISTANCE_FROM_CENTER],[-1/2, -np.sqrt(3)/2, - DISTANCE_FROM_CENTER],[-1/2, np.sqrt(3)/2, - DISTANCE_FROM_CENTER]])

class Driver(Node):
    def __init__(self):
        super().__init__('velocity_to_angle_node')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.callback,
            10)
        self.serial = serial.Serial('/dev/ttyACM0', 115200)
        self.publisher = self.create_publisher(Odometry, 'odom', 10)
        self.prev_x = 0
        self.prev_y = 0
        self.prev_t = 0

    def callback(self, msg):
        # Convert velocity commands to wheel speeds for a three-wheeled omnidirectional robot
        linear_velocity_x = msg.linear.x
        linear_velocity_y = msg.linear.y
        angle = math.atan2(linear_velocity_y, linear_velocity_x)

        # Send angle over serial and receive encoder data
        data = str(angle)
        self.serial.write(data.encode())
        encoder_data = self.serial.readline().decode().strip().split(',')
        # Parse encoder data and calculate odometry values for a three-wheeled omnidirectional robot
        u_1, u_2, u_3 = map(int, encoder_data)
        U = np.array([u_1, u_2, u_3])
        velocity = np.dot(np.linalg.inv(SYSTEM_MATRIX),U)
        v_x = velocity[0]
        v_y = velocity[1]
        t = current_time = self.get_clock().now()
        x = self.prev_x + (t-self.prev_t)*v_x
        y = self.prev_y + (t-self.prev_t)*v_y
        odom_msg = Odometry()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        # odom_msg.pose.pose.orientation.x = 0.0
        # odom_msg.pose.pose.orientation.y = 0.0
        # odom_msg.pose.pose.orientation.z = 0.0
        # odom_msg.pose.pose.orientation.w = 1.0
        odom_msg.twist.twist.linear
        # odom_msg.twist.twist.angular.z = 0.0
        self.publisher.publish(odom_msg)
 
def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()




    