import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist interface from the geometry_msgs package
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Exercise31(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('exercise31')
        # create the publisher object
        # in this case, the publisher will publish on /cmd_vel Topic with a queue size of 10 messages.
        # use the Twist module for /cmd_vel Topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))        
        # define the timer period for 0.5 seconds


        timer_period = 0.5

        self.laser_forward = 0

        self.cmd = Twist()
        # create a timer sending two parameters:
        # - the duration between 2 callbacks (0.5 seconds)
        # - the timer function (timer_callback)
        self.timer = self.create_timer(timer_period, self.motion)

    def laser_callback(self, msg):
        self.laser_forward = msg.ranges[359]    

    def motion(self):
        self.get_logger().info('I receive: "%s"' % str(self.laser_forward))
        if(self.laser_forward > 5):
            self.cmd.linear.x = 0.5
            self.cmd.angular.z = 0.5
        elif(self.laser_forward >= 0.5):
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0
        else:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0

        # Publish the message to the Topic
        self.publisher_.publish(self.cmd)
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % self.cmd)
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    exercise31 = Exercise31()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(exercise31)
    # Explicity destroys the node
    exercise31.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()