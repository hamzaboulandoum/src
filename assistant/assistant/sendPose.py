import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class Nav2Client(Node):

    def __init__(self):
        super().__init__('nav2_client')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal = NavigateToPose.Goal()

    def send_goal(self, pose: PoseStamped):
        self.goal.pose = pose
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(self.goal)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result == 0:
            self.get_logger().info('Goal succeeded')
        else:
            self.get_logger().info('Goal failed with error code: {0}'.format(result))


def main(args=None):
    rclpy.init(args=args)

    nav2_client = Nav2Client()

    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = 1.0
    pose.pose.position.y = 2.0
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0

    nav2_client.send_goal(pose)

    rclpy.spin(nav2_client)

    nav2_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
