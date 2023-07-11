import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from actions_quiz_msg.action import Distance
from geometry_msgs.msg import Twist
import time
from rclpy.qos import ReliabilityPolicy, QoSProfile
from nav_msgs.msg import Odometry
import math
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor


class Actions_quiz_server_node(Node):

    def __init__(self):
        super().__init__('actions_quiz_server_node')
        self.cmd = Twist()
        self.inicial_position = None

        self.group1 = ReentrantCallbackGroup()
        self.group2 = ReentrantCallbackGroup()
        self.group3 = ReentrantCallbackGroup()

        self._action_server = ActionServer(self, Distance, 'distance_as',self.execute_callback, callback_group=self.group2) 
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10, callback_group=self.group3)


        self.subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.group1)
    
    
    def odometry_callback(self, msg):
        self.position = msg.pose.pose.position.y
        if self.inicial_position == None:
            self.inicial_position = self.position

        self.distance = abs(self.position - self.inicial_position)
        self.get_logger().info('New distance: ' + str(self.distance))
    
    def execute_callback(self, goal_handle):
        while self.inicial_position == None:
            time.sleep(0.1)
            self.get_logger().info('Waiting for odometry...')

        
        self.get_logger().info('Executing goal...')

        # feedback_msg = Distance.current_dist()
        # feedback_msg.feedback = 5.0

        # feedback_msg.feedback = "Moving to the left left left..."

        for i in range(1, goal_handle.request.seconds):
            self.get_logger().info('Time: ' + str(i))

            self.velocity = 0.3

            # self.get_logger().info('Feedback: {0} '.format(feedback_msg.feedback))

            # goal_handle.publish_feedback(feedback_msg)
            self.cmd.linear.x = self.velocity
            self.cmd.angular.z =0.0

            feedback = Distance.Feedback()
            feedback.current_dist = self.distance
            goal_handle.publish_feedback(feedback)
            
            self.publisher_.publish(self.cmd)
            time.sleep(1)

        result = Distance.Result()

        result.status = True
        result.total_dist = self.distance

        goal_handle.succeed()

        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
            
        self.publisher_.publish(self.cmd)

        return result

# def main(args=None):
#     rclpy.init(args=args)

#     actions_quiz_server_node = Actions_quiz_server_node()

#     rclpy.spin(actions_quiz_server_node)

def main(args=None):
    rclpy.init(args=args)

    actions_quiz_server_node = Actions_quiz_server_node()

    executor = MultiThreadedExecutor()

    executor.add_node(actions_quiz_server_node)

    executor.spin()

    actions_quiz_server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()