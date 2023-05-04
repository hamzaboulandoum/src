import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import TransformException

class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pose_publisher = self.create_publisher(PoseStamped, '/pose', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)

    def publish_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = transform.transform.translation.x
            pose_stamped.pose.position.y = transform.transform.translation.y
            pose_stamped.pose.position.z = transform.transform.translation.z
            pose_stamped.pose.orientation.x = transform.transform.rotation.x
            pose_stamped.pose.orientation.y = transform.transform.rotation.y
            pose_stamped.pose.orientation.z = transform.transform.rotation.z
            pose_stamped.pose.orientation.w = transform.transform.rotation.w
            self.pose_publisher.publish(pose_stamped)
        except TransformException as ex:
            self.get_logger().warn('Transform error: {}'.format(ex))

def main(args=None):
    rclpy.init(args=args)
    pose_publisher_node = PosePublisherNode()
    rclpy.spin(pose_publisher_node)
    pose_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
