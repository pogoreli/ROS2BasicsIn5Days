from geometry_msgs.msg import Twist
from services_quiz_srv.srv import Turn
import rclpy
from rclpy.node import Node

class Services_quiz_server_node(Node):
    def __init__(self):
        super().__init__('services_quiz_server_node')
        self.srv = self.create_service(Turn, 'turn', self.custom_service_callback)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.waitDone = False
        self.angular_velocity = 0.0
        self.time_since_timer_start = 0.0
        self.time_interval = 1
        self.motion_timer = self.create_timer(self.time_interval, self.motion)

    def custom_service_callback(self, request, response):
        direction = request.direction
        coeff = 0

        if direction == "left":
            coeff = 1
            self.get_logger().info('Turning left!!')
        elif direction == "right":
            coeff = -1
            self.get_logger().info('Turning right!!')
        else:
            coeff = 0
            self.get_logger().info('Incorrect value')

        self.angular_velocity = request.angular_velocity * coeff
        
        self.time = request.time

        self.get_logger().info('direction' + direction + 'velocity' + str(self.angular_velocity) + "time" + str(self.time))


        self.waitDone = False
        self.timer = self.create_timer(self.time_interval, self.timer_callback) 

        response.success = True
        self.response = response
        return response

    def timer_callback(self):
        # self.get_logger().info('timer_callback')
        self.time_since_timer_start += self.time_interval

        if self.time_since_timer_start >= self.time:
            self.waitDone = True
            self.response.success = True
            self.timer.cancel()
            self.get_logger().info('Time to stop')
            self.time_since_timer_start = 0
            self.timer.cancel()
        else:
            self.get_logger().info('Still waiting')        
            

    def motion(self):
        msg = Twist()
        msg.linear.x = 0.0
        if not self.waitDone:
            msg.angular.z = self.angular_velocity
            # self.get_logger().info('Rotation in progress')
        else:
            msg.angular.z = 0.0

        self.publisher_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    services_quiz_server_node = Services_quiz_server_node()
    rclpy.spin(services_quiz_server_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
