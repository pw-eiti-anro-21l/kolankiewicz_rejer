import rclpy
import keyboard
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CustomPublisher(Node):

    def __init__(self):

        super().__init__('custom_publisher')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        msg = Twist()
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        if keyboard.is_pressed("q"):
            msg.linear.x = 2.0
            msg.angular.z = 0.0
        if keyboard.is_pressed("w"):
            msg.linear.x = -2.0
            msg.angular.z = 0.0
        if keyboard.is_pressed("e"):
            msg.linear.x = 0.0
            msg.angular.z = 2.0
        if keyboard.is_pressed("r"):
            msg.linear.x = 0.0
            msg.angular.z = -2.0
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    custom_publisher = CustomPublisher()
    rclpy.spin(custom_publisher)
    custom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
