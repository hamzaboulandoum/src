import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class Nav2Client(Node):

    def __init__(self):
        super().__init__('nav2_client')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal = NavigateToPose.Goal()
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal',
            self.goal_callback,
            10
        )

    def goal_callback(self, msg: PoseStamped):
        self.send_goal(msg)

    def send_goal(self, pose: PoseStamped):
        self.goal.pose = pose.pose
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

    rclpy.spin(nav2_client)

    nav2_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
