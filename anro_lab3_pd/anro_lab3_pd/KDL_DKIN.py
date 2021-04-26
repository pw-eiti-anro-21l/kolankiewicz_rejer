import rclpy
import math
import mathutils
import yaml
import os
import PyKDL as kdl
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.clock import ROSClock
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory


class KDL_DKIN(Node):

    def __init__(self):
        super().__init__('kdl_dkin')
        file_name = 'dh_matrix.txt'
        path = os.path.join( get_package_share_directory('anro_lab3_pd'), file_name)
        self.rows = self.read_txt(path)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        chain_test = self.kdl_chain()
        joints= kdl.JntArray(3)
        joints[0] = msg.position[0]
        joints[1] = msg.position[1]
        joints[2] = msg.position[2]
        fk = kdl.ChainFkSolverPos_recursive(chain_test)
        frame = kdl.Frame()
        fk.JntToCart(joints, frame)
        quat1 = frame.M.GetQuaternion()
        xyz = frame.p
        print("kwat")
        print(quat1)
        # quat1 = T.to_quaternion()
        # xyz=T.to_translation()
        qos_profile = QoSProfile(depth=10)
        pose_publisher = self.create_publisher(PoseStamped, '/kdl_pose', qos_profile)
        pose = PoseStamped()
        pose.header.stamp = ROSClock().now().to_msg()
        pose.header.frame_id = "base"
        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        pose.pose.orientation.x = quat1[0]
        pose.pose.orientation.y = quat1[1]
        pose.pose.orientation.z = quat1[2]
        pose.pose.orientation.w = quat1[3]
        pose_publisher.publish(pose)
        print(pose)

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

    def kdl_chain(self):
        final_chain = kdl.Chain()
        a1, d1, alfa1, theta1 = self.rows[0]
        a2, d2, alfa2, theta2 = self.rows[1]
        a3, d3, alfa3, theta3 = self.rows[2]

        joint1 = kdl.Joint(kdl.Joint.TransZ)
        dh1 = kdl.Frame().DH(a2,alfa2,d1,theta1)
        segm1 = kdl.Segment(joint1, dh1)
        final_chain.addSegment(segm1)
        
        joint2 = kdl.Joint(kdl.Joint.RotZ)
        dh2 = kdl.Frame().DH(a3,alfa3,d2,theta2)
        segm2 = kdl.Segment(joint2, dh2)
        final_chain.addSegment(segm2)
        
        joint3 = kdl.Joint(kdl.Joint.RotZ)
        dh3 = kdl.Frame().DH(0,0,d3,theta3)
        segm3 = kdl.Segment(joint3, dh3)
        final_chain.addSegment(segm3)
        return final_chain


def main(args=None):
    rclpy.init(args=args)
    kdl_dkin = KDL_DKIN()
    rclpy.spin(kdl_dkin)
    kdl_dkin.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
