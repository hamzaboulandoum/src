import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
from nav2d_utils import astar_path

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.start_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.get_logger().info('Path planner node started')

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.map_ = None
        
        self.tf_buffer = Buffer(self.get_clock())
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)
        self.publisher = self.create_publisher(Path, '/path', 10)

    def goal_callback(self, msg):
        self.goal_pose = msg
        path_msg = self.compute_path()
        self.publisher.publish(path_msg)

    def map_callback(self, msg):
        self.map_ = msg  # Store the received map message

    def get_map(self):
        while self.map_ is None:
            self.get_logger().info('Waiting for map...')
        return self.map_


    def compute_path(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',  # target frame
                'map',        # source frame
                self.get_clock().now(), # time (use latest)
                timeout=rclpy.duration.Duration(seconds=1.0)) # timeout
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn('Failed to get transform from map to base_link: {}'.format(e))
            return

        # Update the starting pose of the path planner using the transform
        self.start_pose.pose.position.x = transform.transform.translation.x
        self.start_pose.pose.position.y = transform.transform.translation.y
        self.start_pose.pose.position.z = transform.transform.translation.z
        self.start_pose.pose.orientation = transform.transform.rotation

        # Convert start_pose and goal_pose to (x, y, theta) tuples
        start = (self.start_pose.pose.position.x, self.start_pose.pose.position.y, 0)
        goal = (self.goal_pose.pose.position.x, self.goal_pose.pose.position.y, 0)

        # Compute the A* path using the astar_path method from nav2d_utils
        # The astar_path method takes a start and goal pose in (x, y, theta) format, a resolution, a map, and a maximum distance
        # The returned path is a list of (x, y, theta) tuples
        resolution = 0.1  # meters per pixel
        max_distance = 100.0  # maximum distance (in meters) to consider for path planning
        path = astar_path(start, goal, resolution, self.get_map(), max_distance)

        # Convert the path from a list of (x, y, theta) tuples to a Path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        for p in path:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = path_msg.header.frame_id
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        return path_msg        

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()