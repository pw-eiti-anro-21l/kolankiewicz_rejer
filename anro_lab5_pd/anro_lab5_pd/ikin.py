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
import os

class IKIN(Node):

    def __init__(self):
        super().__init__('ikin')
        self.subscription = self.create_subscription(PoseStamped, 'oint_pose', self.listener_callback, 10)
        qos_profile1 = QoSProfile(depth=10)
        self.publisher = self.create_publisher(JointState, 'joint_states', qos_profile1)
        self.joint_state = JointState()
        self.joint_state.name = ['base_to_first_link', 'first_link_to_second_link', 'second_link_to_tool']
        
        self.file_name = 'dh_matrix.txt'
        self.dh_path = os.path.join( get_package_share_directory('anro_lab5_pd'), self.file_name)
        self.rows = self.read_txt(self.dh_path)
        self.d = self.rows[0][1]
        self.a1 = self.rows[2][0]
        self.a2 = self.rows[3][0]

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
            if joint_1 < -self.d:
                raise ValueError
            self.joint_state.header.stamp = ROSClock().now().to_msg()
            self.joint_state.position = [joint_1, joint_2, joint_3]
            self.publisher.publish(self.joint_state)
        except ValueError:
            self.get_logger().error("Position out of reach!")

    def read_txt(self, file_path):
        file = open(file_path)
        lines = file.readlines()[1:]
        rows = []
        for line in lines:
            row = line.split()
            row = [float(item) for item in row]
            rows.append(row)
        file.close()
        return rows


def main(args=None):
    rclpy.init(args=args)
    ikin = IKIN()
    rclpy.spin(ikin)
    ikin.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
