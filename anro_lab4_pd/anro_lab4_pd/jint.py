from lab4_interfaces.srv import Jintstructure  
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
import time
import math


class Jint(Node):

    def __init__(self):
        super().__init__('jint')
        self.beg_position = [0.0,0.0,0.0]
        self.srv = self.create_service(Jintstructure, 'jint_structure', self.jint_control_srv_callback)        # CHANGE
        self.subscriber = self.create_subscription(JointState, 'joint_states_sub',self.listener_callback,1)
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(JointState, 'joint_pub', qos_profile)


    def jint_structure_callback(self, request, response):
        response = ""
        return response

    def listener_callback(self, msg):
        self.beg_position[0] = msg.position[0]
        self.beg_position[1] = msg.position[1]
        self.beg_position[2] = msg.position[2]

    def jint_control_srv_callback(self, request, response ):
        self.linear(request)
        response.resp = "Interpolation succeded"
        return response


    def linear(self, request):
        start = self.beg_position
        rate_given = 10
        steps = math.floor(request.time*rate_given)
        now = self.get_clock().now()
        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['prism', 'cont1', 'cont2']
        joint_state.position = start
        curr_pos =start
        delta = [0.0,0.0,0.0]
        delta[0]=(request.prismatic - start[0])/steps
        delta[1]=(request.angle1 - start[1])/steps
        delta[2]=(request.angle2 - start[2])/steps
        for i in range(0, steps):
            for n in range(0,3):
                curr_pos[n]+=delta[n]
            joint_state.position = curr_pos
            print(joint_state)
            self.publisher.publish(joint_state)
            time.sleep(1/rate_given)

def main(args=None):
    rclpy.init(args=args)

    jint = Jint()

    rclpy.spin(jint)

    rclpy.shutdown()

if __name__ == '__main__':
    main()