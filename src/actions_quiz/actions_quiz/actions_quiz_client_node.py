import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from actions_quiz_msg.action import Distance


class MyActionClient(Node):

    def __init__(self):
        super().__init__('actions_quiz_client_node')
        self._action_client = ActionClient(self, Distance, 'distance_as')

    def send_goal(self, time):
        goal_msg = Distance.Goal()
        goal_msg.seconds = time

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.status))
        self.get_logger().info('total_dist: {0}'.format(result.total_dist))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Received feedback: {0}'.format(feedback.current_dist))


def main(args=None):
    rclpy.init(args=args)

    action_client = MyActionClient()

    action_client.send_goal(5)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()