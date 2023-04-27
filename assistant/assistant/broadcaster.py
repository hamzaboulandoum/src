import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from math import sin, cos
import serial
import numpy as np

#constants
WHEEL_RADIUS = 0.05
DISTANCE_FROM_CENTER = 0.18
CORR_CONST = 0.15
SYSTEM_MATRIX = 1/WHEEL_RADIUS*np.array([[1, 0, - DISTANCE_FROM_CENTER],[-1/2, -np.sqrt(3)/2, - DISTANCE_FROM_CENTER],[-1/2, np.sqrt(3)/2, - DISTANCE_FROM_CENTER]])
V = 0.18
class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 50)
        self.broadcaster_ = TransformBroadcaster(self)

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vx, self.vy, self.vth = 0.0, 0.0, 0.0

        self.current_time = self.get_clock().now()
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(0.02, self.timer_callback)

        self.declare_parameter('serial_port', value="/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0")
        self.serial_port = self.get_parameter('serial_port').value

        self.declare_parameter('baud_rate', value=9600)
        self.baud_rate = self.get_parameter('baud_rate').value

        self.serial = serial.Serial(self.serial_port, self.baud_rate)

    def timer_callback(self):
        encoder_data = self.serial.readline().decode().strip().split(', ')
        while len(encoder_data) < 3 :
            encoder_data = self.serial.readline().decode().strip().split(', ')

        #self.get_logger().info(str(encoder_data))

        # Parse encoder data and calculate odometry values for a three-wheeled omnidirectional robot
        u_1, u_2, u_3 = map(float, encoder_data)
        self.get_logger().info(str(u_1) +' , ' + str(u_2)+' , ' + str(u_3))
        U = np.array([u_1, u_2, u_3])
        velocity = np.dot(np.linalg.inv(SYSTEM_MATRIX),U)
        v_x = velocity[0]
        v_y = velocity[1]
        self.get_logger().info(str(v_x) +' , ' + str(v_y))
        self.vx = v_x
        self.vy = v_y
        self.vth = 0.0
        

        self.current_time = self.get_clock().now()

        dt = (self.current_time - self.last_time).nanoseconds / 1e9
        delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
        delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        odom_quat = Quaternion()
        odom_quat.z = sin(self.th/2.0)
        odom_quat.w = cos(self.th/2.0)

        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.current_time.to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'

        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = odom_quat

        self.broadcaster_.sendTransform(odom_trans)

        odom = Odometry()
        odom.header.stamp = self.current_time.to_msg()
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat

        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth

        self.publisher_.publish(odom)

        self.last_time = self.current_time

def main(args=None):
    rclpy.init(args=args)

    odom_publisher = OdometryPublisher()

    rclpy.spin(odom_publisher)
    odom_publisher.serial.close()
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()