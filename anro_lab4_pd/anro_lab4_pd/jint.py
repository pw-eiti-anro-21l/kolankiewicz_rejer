from lab4_interfaces.srv import Jintstructure  
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
import time
import math
from rclpy.clock import ROSClock
#from threading import Timer


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
        self.rate=10


    def jint_control_srv_callback(self, request, response ):
        if request.method == "linear":
            self.linear(request)
        elif request.method == "polynomial":
            self.polynomial(request)
        else:
            print('Wrong method')
        response.resp = "Interpolation succeded"
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
            self.publisher.publish(self.joint_state)
            time.sleep(1/self.rate)


def polynomial(self, request):

    return
        

def main(args=None):
    rclpy.init(args=args)

    jint = Jint()

    rclpy.spin(jint)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
