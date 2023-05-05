import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan

class PathFollowerObstacleNode(Node):
    def __init__(self):
        super().__init__('path_follower_obstacle')
        self.create_subscription(Path, '/path', self.path_callback, 10)
        self.create_subscription(Path, '/pose', self.pose_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_pose_ = None
        self.current_segment_ = 0
        self.path_ = None
        self.obstacle_threshold_ = 0.2  # Distance threshold for stopping when an obstacle is close enough
    
    def scan_callback(self, msg):
        # Check if there is an obstacle close enough to stop the robot
        if msg.ranges:
            if min(msg.ranges) < self.obstacle_threshold_:
                twist = Twist()
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.publisher_.publish(twist)

    def path_callback(self, msg):
        self.path_ = msg  # Store the received path message
        self.current_segment_ = 0  # Reset the current path segment to the first one

    def pose_callback(self, msg):
        self.current_pose_ = msg  # Store the received pose message
        if self.path_ is not None:
            self.follow_path()  # If a path has been received, follow it

    def scan_callback(self, msg):
        # Check if there is an obstacle close enough to stop the robot
        if msg.ranges:
            if min(msg.ranges) < self.obstacle_threshold_:
                twist = Twist()
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.publisher_.publish(twist)

    def follow_path(self):
        # Get the current path segment, i.e. the segment between the current pose and the next waypoint on the path
        start_pose = self.current_pose_
        end_pose = self.path_.poses[self.current_segment_ + 1]
        segment = (start_pose.pose.position.x, start_pose.pose.position.y), (end_pose.pose.position.x, end_pose.pose.position.y)

        # Compute the required velocity to follow the path segment using a proportional controller
        # The velocity is proportional to the distance to the next waypoint
        kp = 0.5  # Proportional gain
        distance = ((segment[1][0] - segment[0][0])**2 + (segment[1][1] - segment[0][1])**2)**0.5
        velocity = kp * distance

        # Compute the required angular velocity to turn towards the next waypoint
        # The angular velocity is proportional to the difference between the current orientation and the target orientation
        # We assume that the robot only needs to turn around the z-axis (yaw)
        target_orientation = math.atan2(segment[1][1] - segment[0][1], segment[1][0] - segment[0][0])
        current_orientation = self.current_pose_.pose.orientation.z
        ka = 0.5  # Proportional gain for angular velocity
        angular_velocity = ka * (target_orientation - current_orientation)

        # Create a Twist message with the computed velocities and publish it
        twist = Twist()
        twist.linear.y = - velocity
        twist.angular.z = angular_velocity
        self.publisher_.publish(twist)

        # If the current pose is close enough to the next waypoint, move to the next path segment
        if distance < 0.1:
            self.current_segment_ += 1

            # If we have reached the end of the path, stop the robot
            if self.current_segment_ >= len(self.path_.poses) - 1:
                twist.linear.y= 0.0
                twist.angular.z = 0.0
                self.publisher_.publish(twist)


