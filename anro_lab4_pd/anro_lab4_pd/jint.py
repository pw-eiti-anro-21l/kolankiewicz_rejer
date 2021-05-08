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
        self.beg_position = [0.0,0.0,0.0]
        self.srv = self.create_service(Jintstructure, 'jint_structure', self.jint_control_srv_callback)        # CHANGE
        qos_profile1 = QoSProfile(depth=10)
        self.publisher = self.create_publisher(JointState, 'joint_states', qos_profile1)


    def jint_control_srv_callback(self, request, response ):
        self.linear(request)
        response.resp = "Interpolation succeded"
        return response


    def linear(self, request):
        start = self.beg_position
        rate_given = 10
        steps = math.floor(request.time*rate_given)
        print(steps)
        joint_state = JointState()
        joint_state.header.stamp = ROSClock().now().to_msg()
        joint_state.name = ['base_to_first_link', 'first_link_to_second_link', 'second_link_to_tool']
        joint_state.position = start
        curr_pos =start
        delta = [0.0,0.0,0.0]
        delta[0]=(request.prismatic - start[0])/steps
        delta[1]=(request.angle1 - start[1])/steps
        delta[2]=(request.angle2 - start[2])/steps
        for i in range(0, steps):
            print(i)
            for n in range(0,3):
                curr_pos[n]+=delta[n]
            joint_state.position = curr_pos
            print(joint_state)
            self.publisher.publish(joint_state)
            time.sleep(0.1)
        self.beg_position=curr_pos
        print(curr_pos)
        

def main(args=None):
    rclpy.init(args=args)

    jint = Jint()

    rclpy.spin(jint)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
