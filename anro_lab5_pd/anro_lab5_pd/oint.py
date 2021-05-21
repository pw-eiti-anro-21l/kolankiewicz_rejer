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


class Oint(Node):

    def __init__(self):
        super().__init__('oint')
        self.srv = self.create_service(Ointstructure, 'oint_structure', self.oint_control_srv_callback)        # CHANGE
        qos_profile_pose = QoSProfile(depth=10)
        self.pose_publisher = self.create_publisher(PoseStamped, 'oint_pose', qos_profile_pose)
        self.pose = PoseStamped()
        self.pose.header.stamp = ROSClock().now().to_msg()
        self.pose.header.frame_id = "base"

        self.pose.pose.position = Point(x=0.0,y=0.0,z=0.0)
        self.pose.pose.orientation = Quaternion(w=0.0,x=0.0,y=0.0,z=0.0)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.last_rpy = [0.0, 0.0, 0.0]
        self.rate = 20
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


    def oint_control_srv_callback(self, request, response ):
        if request.method == "linear":
            self.linear(request)
            response.resp = "Linear interpolation succeded"
        elif request.method == "polynomial":
            self.polynomial(request)
            response.resp = "Polynomial interpolation succeded"
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

    def linear(self, request):
        steps = math.floor(request.time*self.rate)
        delta = [0.0,0.0,0.0]
        quat_delta = [0.0,0.0,0.0,0.0]
        delta[0]=(request.x - self.pose.pose.position.x)/steps
        delta[1]=(request.y - self.pose.pose.position.y)/steps
        delta[2]=(request.z - self.pose.pose.position.z)/steps
        rpy = [request.roll, request.pitch, request.yaw]
        delta_rpy = [0.0, 0.0, 0.0]
        delta_rpy[0] = (request.roll - self.last_rpy[0])/steps
        delta_rpy[1] = (request.pitch - self.last_rpy[1])/steps
        delta_rpy[2] = (request.yaw - self.last_rpy[2])/steps
        for i in range(0, steps):
            self.pose.pose.position.x+=delta[0]
            self.pose.pose.position.y+=delta[1]
            self.pose.pose.position.z+=delta[2]
            self.roll += delta_rpy[0]
            self.pitch += delta_rpy[1]
            self.yaw += delta_rpy[2]
            self.last_rpy = [self.roll, self.pitch, self.yaw]
            updated_quat = self.rpy_to_quat(self.last_rpy)
            self.pose.pose.orientation.w = updated_quat[0]
            self.pose.pose.orientation.x = updated_quat[1]
            self.pose.pose.orientation.y = updated_quat[2]
            self.pose.pose.orientation.z = updated_quat[3]
            self.pose.header.stamp = ROSClock().now().to_msg()
            self.pose_publisher.publish(self.pose)
            self.marker_oint()
            time.sleep(1/self.rate)

    def polynomial(self, request):
        steps = math.floor(request.time*self.rate)
        a = [0.0,0.0,0.0]
        b = [0.0,0.0,0.0]
        beg = [self.pose.pose.position.x, 
                self.pose.pose.position.y, 
                self.pose.pose.position.z]
        distances = [request.x - self.pose.pose.position.x,
                    request.y - self.pose.pose.position.y,
                    request.z - self.pose.pose.position.z]
        rpy_a = [0.0,0.0,0.0]
        rpy_b = [0.0,0.0,0.0]
        rpy_beg = self.last_rpy
        rpy_distances = [request.roll - self.roll,
                        request.pitch - self.pitch,
                        request.yaw - self.yaw]
        for k in range(3):
            a[k] = -2*distances[k]/pow(steps, 3)
            b[k] = 3*distances[k]/pow(steps, 2)
            rpy_a[k] = -2*rpy_distances[k]/pow(steps, 3)
            rpy_b[k] = 3*rpy_distances[k]/pow(steps, 2)
        for i in range(steps):
            self.pose.pose.position.x = a[0]*pow(i,3) + b[0]*pow(i,2) + beg[0]
            self.pose.pose.position.y = a[1]*pow(i,3) + b[1]*pow(i,2) + beg[1]
            self.pose.pose.position.z = a[2]*pow(i,3) + b[2]*pow(i,2) + beg[2]
            self.roll = rpy_a[0]*pow(i,3) + rpy_b[0]*pow(i,2) + rpy_beg[0]
            self.pitch = rpy_a[1]*pow(i,3) + rpy_b[1]*pow(i,2) + rpy_beg[1]
            self.yaw = rpy_a[2]*pow(i,3) + rpy_b[2]*pow(i,2) + rpy_beg[2]
            self.last_rpy = [self.roll, self.pitch, self.yaw]
            updated_quat = self.rpy_to_quat(self.last_rpy)
            self.pose.pose.orientation.w = updated_quat[0]
            self.pose.pose.orientation.x = updated_quat[1]
            self.pose.pose.orientation.y = updated_quat[2]
            self.pose.pose.orientation.z = updated_quat[3]
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



def main(args=None):
    rclpy.init(args=args)

    oint = Oint()
    
    rclpy.spin(oint)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

