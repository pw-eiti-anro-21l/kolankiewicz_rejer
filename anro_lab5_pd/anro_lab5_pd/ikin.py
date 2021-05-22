import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from rclpy.clock import ROSClock
from ament_index_python.packages import get_package_share_directory

class IKIN(Node):

    def __init__(self):
        super().__init__('ikin')
        self.subscription = self.create_subscription(PoseStamped, 'oint_pose', self.listener_callback, 10)
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_state = JointState()
        self.a1 = 1.5
        self.a2 = 1.5

    def listener_callback(self, msg):
        try:
            x_end = msg.pose.position.x
            y_end = msg.pose.position.y
            z_end = msg.pose.position.z
            joint_1, joint_2, joint_3 = 0, 0, 0

            s2 = (x_end - y_end)/self.a2
            c2 = math.sqrt(1 - s2**2)
            theta2 = math.atan2(s2, c2)

            s112 = (x_end**2 + y_end**2 - self.a1**2 - self.a2**2)/(2*self.a2*self.a1)
            c112 = math.sqrt(1 - s112**2)
            theta112 = math.atan2(s112, c112)
            theta1 = (theta112-theta2)/2

            joint_1 = z_end
            joint_2 = theta1
            joint_3 = theta2

            self.joint_state.position = [joint_1, joint_2, joint_3]
            self.publisher.publish(self.joint_state)
        except ValueError:
            print("Position out of reach!")


def main(args=None):
    rclpy.init(args=args)
    ikin = IKIN()
    rclpy.spin(ikin)
    ikin.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()