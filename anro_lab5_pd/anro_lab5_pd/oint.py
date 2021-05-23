from lab5_interfaces.srv import Ointstructure  
from rclpy.qos import QoSProfile
import rclpy
from rclpy.node import Node
import time
import math
from rclpy.clock import ROSClock
import mathutils
from geometry_msgs.msg import Quaternion, Point, PoseStamped,Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import os
from ament_index_python.packages import get_package_share_directory



class Oint(Node):

    def __init__(self):
        super().__init__('oint')
        self.srv = self.create_service(Ointstructure, 'oint_structure', self.oint_control_srv_callback)        # CHANGE
        qos_profile_pose = QoSProfile(depth=10)
        self.pose_publisher = self.create_publisher(PoseStamped, 'oint_pose', qos_profile_pose)
        self.pose = PoseStamped()
        self.pose.header.stamp = ROSClock().now().to_msg()
        self.pose.header.frame_id = "base"

        self.pose.pose.position = Point(x=0.0,y=0.0, z=0.0)
        self.pose.pose.orientation = Quaternion(w=0.0,x=0.0,y=0.0,z=0.0)
        self.rate = 20
        qos_profile_marker = QoSProfile(depth=10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'marker_array', qos_profile_marker)
        self.marker_init()
        self.y = 1.0

        self.file_name = 'dh_matrix.txt'
        self.dh_path = os.path.join( get_package_share_directory('anro_lab5_pd'), self.file_name)
        self.rows = self.read_txt(self.dh_path)
        self.d = self.rows[0][1]
        self.a1 = self.rows[2][0]
        self.a2 = self.rows[3][0]
        print(self.d)
        print(self.a1)
        print(self.a2)

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


    def oint_control_srv_callback(self, request, response ):
        if request.method == "rectangle":
            self.rectangle(request)
            response.resp = "Rectangle interpolation succeded"
        elif request.method == "ellipse":
            self.ellipse(request)
            response.resp = "Ellipse interpolation succeded"
        else:
            response.resp = 'Wrong method'
        return response

    def marker_oint(self):
        self.marker.pose.position= self.pose.pose.position
        self.markerArray.markers.append(self.marker)
        id = 0
        for m in self.markerArray.markers:
            m.id = id
            id += 1
        self.marker_publisher.publish(self.markerArray)

    def linear(self, x,y,z,t):
        steps = math.floor(t*self.rate)
        delta = [0.0,0.0,0.0]
        delta[0]=(x - self.pose.pose.position.x)/steps
        delta[1]=(y - self.pose.pose.position.y)/steps
        delta[2]=(z - self.pose.pose.position.z)/steps
        for i in range(0, steps):
            self.pose.pose.position.x+=delta[0]
            self.pose.pose.position.y+=delta[1]
            self.pose.pose.position.z+=delta[2]
            self.pose.header.stamp = ROSClock().now().to_msg()
            self.pose_publisher.publish(self.pose)
            self.marker_oint()
            time.sleep(1/self.rate)
        
    def ellipse(self,request):
        self.linear(request.x,self.y,0,2)# returning to to starting position
        x=request.x
        z=request.z
        steps = math.floor(request.time*self.rate)
        t=0
        for i in range(steps):
            self.pose.pose.position.x = x * math.cos(t)
            self.pose.pose.position.z = z * math.sin(t)
            self.pose.pose.position.y = self.y
            t+=1/self.rate
            self.pose.header.stamp = ROSClock().now().to_msg()
            self.pose_publisher.publish(self.pose)
            self.marker_oint()
            time.sleep(1/self.rate)

    def rectangle(self,request):
        self.linear(0,self.y,0.5,1)# returning to to starting position
        x=request.x
        z=request.z
        steps = math.floor(request.time*self.rate)
        t=0
        delta=0.05
        delta_x=delta
        delta_z=-delta
        steps_x = math.floor(x/delta)
        if steps_x % 2 ==0:
            steps_x+=1
        steps_z =math.floor(z/delta) 

        for i in range(steps):
            current_cycle = (i-(steps_x-1)/2)%(steps_x+steps_z)
            if current_cycle < 0:
                self.pose.pose.position.x+=delta_x
            elif current_cycle ==0:
                self.pose.pose.position.x+=delta_x
                delta_x=-delta_x
            elif current_cycle < steps_z:
                self.pose.pose.position.z+=delta_z
            elif current_cycle == steps_z:
                self.pose.pose.position.z+=delta_z
                delta_z=-delta_z
            else:
                self.pose.pose.position.x+=delta_x
            self.pose.pose.position.y = self.y
            t+=1/self.rate
            self.pose.header.stamp = ROSClock().now().to_msg()
            self.pose_publisher.publish(self.pose)
            self.marker_oint()
            time.sleep(1/self.rate)
            

    def rpy_to_quat(self, rpy):
        rotZ = mathutils.Matrix.Rotation(rpy[2], 4, 'Z')
        rotY = mathutils.Matrix.Rotation(rpy[1], 4, 'Y')
        rotX = mathutils.Matrix.Rotation(rpy[0], 4, 'X')
        T = rotZ@rotY@rotX
        quat = T.to_quaternion()
        return quat

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

    oint = Oint()
    
    rclpy.spin(oint)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
