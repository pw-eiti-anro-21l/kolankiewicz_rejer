import rclpy
import mathutils
import yaml
import math
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from rclpy.clock import ROSClock
import os
from ament_index_python.packages import get_package_share_directory



class NONKDL_DKIN(Node):

    def __init__(self):
        super().__init__('nonkdl_dkin')
        file_name = 'dh_matrix.txt'
        path = os.path.join( get_package_share_directory('anro_lab3_pd'), file_name)
        self.rows = self.read_txt(path)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):

        T = self.kinematic(path,msg)
        print(T)
        quat1 = T.to_quaternion()
        print("kwat")
        print(quat1)
        xyz=T.to_translation()
        qos_profile = QoSProfile(depth=10)
        pose_publisher = self.create_publisher(PoseStamped, '/nonkdl_pose', qos_profile)
        pose = PoseStamped()
        pose.header.stamp = ROSClock().now().to_msg()
        pose.header.frame_id = "base"
        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        pose.pose.orientation = Quaternion(w=quat1[0], x=quat1[1], y=quat1[2], z=quat1[3])
        pose_publisher.publish(pose)

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

    def create_t_matrix(self, a, d, alfa, theta):
        rotX = mathutils.Matrix.Rotation(alfa, 4,  'X')
        transX = mathutils.Matrix.Translation((a, 0, 0))
        rotZ = mathutils.Matrix.Rotation(theta, 4, 'Z')
        transZ = mathutils.Matrix.Translation((0, 0, d))
        T = rotX @ transX @ rotZ @ transZ
        return T

    def kinematic(self, msg):
        M01=mathutils.Matrix.Translation((0, 0, 0))
        M12=mathutils.Matrix.Translation((0, 0, 0))
        M23=mathutils.Matrix.Translation((0, 0, 0))
        for i in range(len(self.rows)):
            a, d, alfa, theta = self.rows[i]
            if i == 0:
                d = d+msg.position[0]
                M01 = self.create_t_matrix(a, d, alfa, theta)
            elif i == 1:
                theta += msg.position[1]
                M12 = self.create_t_matrix(a, d, alfa, theta)
            else:
                theta += msg.position[2]-math.pi/2
                M23 = self.create_t_matrix(a, d, alfa, theta)
        return M01 @ M12 @ M23


def main(args=None):
    rclpy.init(args=args)
    nonkdl_dkin = NONKDL_DKIN()
    rclpy.spin(nonkdl_dkin)
    nonkdl_dkin.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
