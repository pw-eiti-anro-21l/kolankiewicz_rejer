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
        qos_profile1 = QoSProfile(depth=10)
        self.publisher = self.create_publisher(JointState, 'joint_states', qos_profile1)
        self.joint_state = JointState()
        self.joint_state.name = ['base_to_first_link', 'first_link_to_second_link', 'second_link_to_tool']
        self.a1 = 1.5
        self.a2 = 1.5
        self.d = 1

    def listener_callback(self, msg):
        try:
            x_end = msg.pose.position.x
            y_end = msg.pose.position.y
            z_end = msg.pose.position.z
            joint_1, joint_2, joint_3 = 0, 0, 0
            
            c2 = (x_end**2+y_end**2-self.a1**2-self.a2**2)/(2*self.a1*self.a2)
            s2 = math.sqrt(1 - c2**2)
            theta2 = math.atan2(s2, c2)
            r = math.sqrt(x_end**2+y_end**2)
            c1=(self.a1**2-self.a2**2+r**2)/(2*self.a1*r)
            s1 = math.sqrt(1-c1**2)
            theta1 = math.atan2(s1,c1)+math.atan2(y_end,x_end)
            
            joint_1 = z_end-self.d
            joint_2 = theta1
            joint_3 = -theta2
            self.joint_state.header.stamp = ROSClock().now().to_msg()
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