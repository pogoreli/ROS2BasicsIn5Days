import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist interface from the geometry_msgs package
from geometry_msgs.msg import Twist
# nav_msgs/msg/Odometry
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np
from geometry_msgs.msg import Quaternion


class Topics_quiz_node(Node):

    def __init__(self):
        super().__init__('topics_quiz_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))        
        
        self.subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    
        
        timer_period = 0.5
        self.laser_forward = 0
        self.laser_left = 0
        self.laser_right = 0
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.motion)
        self.odometry = 0
        self.FSMstate = "S0"
        self.FSMnext = "S0"
        
    def euler_from_quaternion(self, x, y, z, w):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Below should be replaced when porting for ROS2 Python tf_conversions is done.
        """

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return yaw

    def FSM_S0(self):
        self.cmd.linear.x = 0.5
        self.cmd.angular.z = 0.0

    def FSM_S1(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.5

    def FSM_S2(self):
        self.cmd.linear.x = 0.5
        self.cmd.angular.z = 0.0

    def FSM_S3(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0

    def laser_callback(self, msg):
        self.laser_forward = msg.ranges[359]
        self.laser_left = msg.ranges[90]  
        self.laser_right = msg.ranges[270]
        # self.get_logger().info("distance left: " + str(self.laser_left))

    def odometry_callback(self, msg):
        quaternion = msg.pose.pose.orientation
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        self.angle = np.degrees(self.euler_from_quaternion(x, y, z, w))


        self.position = msg.pose.pose.position.y
        # self.get_logger().info("position: " + str(self.position))


    def motion(self):
        if self.FSMstate == "S0":
            self.FSM_S0()
        elif self.FSMstate == "S1":
            self.FSM_S1()
        elif self.FSMstate == "S2":
            self.FSM_S2()
        elif self.FSMstate == "S3":
            self.FSM_S3()

        self.FSMstate = self.FSMnext

        if self.FSMstate == "S0" and self.laser_left >= 5.0:
            self.FSMnext = "S1"
        elif self.FSMstate == "S0" and self.laser_left < 5.0:
            self.FSMnext = "S0"
        elif self.FSMstate == "S1" and self.angle >= 50.0:
            self.FSMnext = "S2"
        elif self.FSMstate == "S1" and self.angle < 50.0:
            self.FSMnext = "S1"
        elif self.FSMstate == "S2" and self.laser_left <= 5.0 or self.laser_right <=5.0:
            self.FSMnext = "S3"
        elif self.FSMstate == "S2" and (self.laser_left > 5.0 or self.laser_right > 5.0):
            self.FSMnext = "S2"
        elif self.FSMstate == "S3":
            self.FSMnext = "S3"

        self.get_logger().info(self.FSMstate)

        # Publish the message to the Topic
        self.publisher_.publish(self.cmd)
            
def main(args = None):
    # self.current_state = "S0"
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    topics_quiz_node = Topics_quiz_node()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(topics_quiz_node)
    # Explicity destroys the node
    topics_quiz_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()