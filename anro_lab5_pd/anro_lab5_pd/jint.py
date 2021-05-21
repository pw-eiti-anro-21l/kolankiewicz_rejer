from lab5_interfaces.srv import Jintstructure  
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
import time
import math
from rclpy.clock import ROSClock
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import PyKDL as kdl
import mathutils
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped
from std_msgs.msg import ColorRGBA



class Jint(Node):

    def __init__(self):
        super().__init__('jint')
        self.srv = self.create_service(Jintstructure, 'jint_structure', self.jint_control_srv_callback)        # CHANGE
        qos_profile1 = QoSProfile(depth=10)
        self.publisher = self.create_publisher(JointState, 'joint_states', qos_profile1)
        self.joint_state = JointState()
        self.joint_state.header.stamp = ROSClock().now().to_msg()
        self.joint_state.name = ['base_to_first_link', 'first_link_to_second_link', 'second_link_to_tool']
        self.joint_state.position = [0.0,0.0,0.0]
        self.rate=20
        qos_profile_marker = QoSProfile(depth=10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'marker_array', qos_profile_marker)
        self.marker_init()


    def marker_init(self):
        self.markerArray = MarkerArray()
        self.marker = Marker()
        self.marker.header.frame_id = "base"

        self.marker.id = 0
        self.marker.action = Marker.DELETEALL
        self.markerArray.markers.append(self.marker)
        self.marker_publisher.publish(self.markerArray)
        
        self.marker.type = self.marker.SPHERE
        self.marker.action = self.marker.ADD
        self.marker.scale = Vector3(x=0.02,y=0.02,z=0.02)
        self.marker.pose.orientation= Quaternion(w=0.0,x=0.0,y=0.0,z=0.0)
        self.marker.color = ColorRGBA(r = 0.4,g = 1.0,b = 0.0,a=1.0)


    def marker_kdl(self):
        self.marker.pose.position= self.KDL_method(self.joint_state).pose.position
        self.markerArray.markers.append(self.marker)
        id = 0
        for m in self.markerArray.markers:
            m.id = id
            id += 1
        self.marker_publisher.publish(self.markerArray)


    def jint_control_srv_callback(self, request, response):
        if request.method == "linear":
            self.linear(request)
            response.resp = "Linear interpolation succeded"
        elif request.method == "polynomial":
            self.polynomial(request)
            response.resp = "Polynomial interpolation succeded"
        else:
            print('Wrong method')
        return response


    def linear(self, request):
        steps = math.floor(request.time*self.rate)
        delta = [0.0,0.0,0.0]
        delta[0]=(request.prismatic - self.joint_state.position[0])/steps
        delta[1]=(request.angle1 - self.joint_state.position[1])/steps
        delta[2]=(request.angle2 - self.joint_state.position[2])/steps
        for i in range(0, steps):
            for n in range(0,3):
                 self.joint_state.position[n]+=delta[n]
            self.joint_state.header.stamp = ROSClock().now().to_msg()
            self.publisher.publish(self.joint_state)
            self.marker_kdl()
            time.sleep(1/self.rate)

    def polynomial(self, request):
        steps = math.floor(request.time*self.rate)
        delta = [0.0,0.0,0.0]
        a = [0.0,0.0,0.0]
        b = [0.0,0.0,0.0]
        beg = [0.0,0.0,0.0]
        distances = [request.prismatic - self.joint_state.position[0],
                    request.angle1 - self.joint_state.position[1],
                    request.angle2 - self.joint_state.position[2]]
        for k in range(3):
            a[k] = -2*distances[k]/pow(steps, 3)
            b[k] = 3*distances[k]/pow(steps, 2)
            beg[k] = self.joint_state.position[k]
        for i in range(steps):
            for n in range(3):
                self.joint_state.position[n] = a[n]*pow(i,3) + b[n]*pow(i,2) + beg[n]
            self.joint_state.header.stamp = ROSClock().now().to_msg()
            self.publisher.publish(self.joint_state)
            self.marker_kdl()
            time.sleep(1/self.rate)


    def KDL_method(self, joints_st):
        file_name = 'dh_matrix.txt'
        path = os.path.join( get_package_share_directory('anro_lab5_pd'), file_name)
        chain_test = self.kdl_chain(path)
        joints= kdl.JntArray(3)
        joints[0] = joints_st.position[0]
        joints[1] = joints_st.position[1]
        joints[2] = joints_st.position[2]
        fk = kdl.ChainFkSolverPos_recursive(chain_test)
        frame = kdl.Frame()
        fk.JntToCart(joints, frame)
        quat1 = frame.M.GetQuaternion()
        xyz = frame.p
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
        return pose

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

    def kdl_chain(self, path_file):
        final_chain = kdl.Chain()
        rows = self.read_txt(path_file)
        a1, d1, alfa1, theta1 = rows[0]
        a2, d2, alfa2, theta2 = rows[1]
        a3, d3, alfa3, theta3 = rows[2]

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

    jint = Jint()

    rclpy.spin(jint)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
