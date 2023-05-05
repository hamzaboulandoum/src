import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan

class StraightLine(Node):
    def __init__(self):
        super().__init__('straight_line')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)
        self.create_subscription(Path, '/pose', self.pose_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_pose_ = None
        self.goal_pose = None
        self.obstacle_threshold_ = 0.2  
    
    def goal_callback(self, msg):
        self.goal_pose = msg

    def scan_callback(self, msg):
        if msg.ranges:
            if min(msg.ranges) < self.obstacle_threshold_:
                twist = Twist()
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.publisher_.publish(twist)


    def pose_callback(self, msg):
        self.current_pose_ = msg  
        if self.goal_pose is not None:
            self.go_to_goal()  

    def scan_callback(self, msg):
        
        if msg.ranges:
            if min(msg.ranges) < self.obstacle_threshold_:
                twist = Twist()
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.publisher_.publish(twist)

    def go_to_goal(self):
       
        start_pose = self.current_pose_
        end_pose = self.goal_pose
        segment = (start_pose.pose.position.x, start_pose.pose.position.y), (end_pose.pose.position.x, end_pose.pose.position.y)

        
        kv = 0.15  
        distance = ((segment[1][0] - segment[0][0])**2 + (segment[1][1] - segment[0][1])**2)**0.5
        velocity = kv * distance

        
      
        target_orientation = math.atan2(segment[1][1] - segment[0][1], segment[1][0] - segment[0][0])
        current_orientation = self.current_pose_.pose.orientation.z
        ka = 0.5 
        angular_velocity = ka * (target_orientation - current_orientation)

       
        twist = Twist()
        twist.linear.y = - velocity
        twist.angular.z = angular_velocity
        self.publisher_.publish(twist)


        if distance < 0.1:
            twist.linear.y= 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.goal_pose = None


