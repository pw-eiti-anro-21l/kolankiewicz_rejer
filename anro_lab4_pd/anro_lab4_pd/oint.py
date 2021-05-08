from lab4_interfaces.srv import Ointstructure  
from rclpy.qos import QoSProfile
import rclpy
from rclpy.node import Node
import time
import math
from rclpy.clock import ROSClock
import mathutils
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
#from threading import Timer


class Oint(Node):

    def __init__(self):
        super().__init__('oint')
        self.srv = self.create_service(Ointstructure, 'oint_structure', self.oint_control_srv_callback)        # CHANGE
        qos_profile_pose = QoSProfile(depth=10)
        self.pose_publisher = self.create_publisher(PoseStamped, 'oint_pose', qos_profile_pose)
        self.pose = PoseStamped()
        self.pose.header.stamp = ROSClock().now().to_msg()
        self.pose.header.frame_id = "base"
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 0.0
        self.pose.pose.orientation.x= 0.0
        self.pose.pose.orientation.y= 0.0
        self.pose.pose.orientation.z= 0.0
        self.pose.pose.orientation.w= 0.0
        self.rate=20


    def oint_control_srv_callback(self, request, response ):
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
        delta[0]=(request.x - self.pose.pose.position.x)/steps
        delta[1]=(request.y - self.pose.pose.position.y)/steps
        delta[2]=(request.z - self.pose.pose.position.z)/steps
        for i in range(0, steps):
            self.pose.pose.position.x+=delta[0]
            self.pose.pose.position.y+=delta[1]
            self.pose.pose.position.z+=delta[2]
            self.pose_publisher.publish(self.pose)
            time.sleep(1/self.rate)

    def polynomial(self, request):
        steps = math.floor(request.time*self.rate)
        delta = [0.0,0.0,0.0]
        a = [0.0,0.0,0.0]
        b = [0.0,0.0,0.0]
        beg = [0.0,0.0,0.0]
        distances = [request.x - self.pose.pose.position.x,
                    request.y - self.pose.pose.position.y,
                    request.z - self.pose.pose.position.z]
        for k in range(3):
            a[k] = -2*distances[k]/pow(steps, 3)
            b[k] = 3*distances[k]/pow(steps, 2)
            if k ==0:
                beg[k] = self.pose.pose.position.x
            elif k == 1:
                beg[k] = self.pose.pose.position.y
            else:
                beg[k] = self.pose.pose.position.z
        for i in range(steps):
            for n in range(3):
                self.pose.pose.position.x = a[0]*pow(i,3) + b[0]*pow(i,2) + beg[0]
                self.pose.pose.position.y = a[1]*pow(i,3) + b[1]*pow(i,2) + beg[1]
                self.pose.pose.position.z = a[2]*pow(i,3) + b[2]*pow(i,2) + beg[2]
            self.pose_publisher.publish(self.pose)
            time.sleep(1/self.rate)

def to_pose(request):
        rotZ = mathutils.Matrix.Rotation(request.yaw, 4, 'Z')
        rotY = mathutils.Matrix.Rotation(request.pitch, 4, 'Y')
        rotX = mathutils.Matrix.Rotation(request.roll, 4, 'X')
        T = rotZ@rotY@rotX
        quat = T.to_quaternion()
        # xyz=T.to_translation()
        self.pose.pose.position.x = request.x
        self.pose.pose.position.y = request.y
        self.pose.pose.position.z = request.z
        self.pose.pose.orientation = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
        #pose.pose.orientation.y = quat1[1]
        #pose.pose.orientation.z = quat1[2]
        #pose.pose.orientation.w = quat1[3]
        self.pose_publisher.publish(pose)


        

def main(args=None):
    rclpy.init(args=args)

    oint = Oint()

    rclpy.spin(oint)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
