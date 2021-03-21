import rclpy
from curtsies import Input
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CustomPublisher(Node):

    def __init__(self):

        super().__init__('custom_publisher')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.1
        self.iterations = 0
        self.start = 0
        self.duration = 10
        self.declare_parameter('forward_key', 'q')
        self.declare_parameter('backward_key', 'w')
        self.declare_parameter('left_key', 'e')
        self.declare_parameter('right_key', 'r')
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.msg = Twist()
        self.clear_msg()

    def clear_msg(self):

        self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = 0.0

    def timer_callback(self):
        forward_key = self.get_parameter("forward_key").get_parameter_value().string_value
        backward_key = self.get_parameter("backward_key").get_parameter_value().string_value
        left_key = self.get_parameter("left_key").get_parameter_value().string_value
        right_key = self.get_parameter("right_key").get_parameter_value().string_value
        with Input(keynames='curtsies') as input_generator:
            c = input_generator.send(0.001)
            if c == forward_key:
                self.msg.linear.x = 2.0
                self.msg.angular.z = 0.0
                self.start = self.iterations
            elif c == backward_key:
                self.msg.linear.x = -2.0
                self.msg.angular.z = 0.0
                self.start = self.iterations
            elif c == left_key:
                self.msg.linear.x = 0.0
                self.msg.angular.z = 2.0
                self.start = self.iterations
            elif c == right_key:
                self.msg.linear.x = 0.0
                self.msg.angular.z = -2.0
                self.start = self.iterations
            elif self.start + self.duration < self.iterations:
                self.clear_msg()
        self.publisher_.publish(self.msg)
        self.iterations += 1


def main(args=None):
    rclpy.init(args=args)
    custom_publisher = CustomPublisher()
    rclpy.spin(custom_publisher)
    custom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
