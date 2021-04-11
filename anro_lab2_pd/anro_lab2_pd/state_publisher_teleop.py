from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from curtsies import Input

class StatePublisherTeleop(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        loop_rate = self.create_rate(30)

        # robot state
        self.prism = -1.
        self.cont1 = 0.
        self.cont2 = 0.
        self.step =0.01
        self.degree_step = pi / 180.0
        self.prism_up_lim = 0.
        self.prism_down_lim = -1.
        self.cont_up_lim = pi
        self.cont_down_lim = -pi
        self.iterations = 0
        self.start = 0
        self.duration = 10
        self.declare_parameter('up_key', 'q')
        self.declare_parameter('down_key', 'w')
        self.declare_parameter('first_left_key', 'e')
        self.declare_parameter('first_right_key', 'r')
        self.declare_parameter('second_left_key', 't')
        self.declare_parameter('second_right_key', 'u')
        self.manage_input()

        # message declarations
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['base_to_first_link', 'first_link_to_second_link', 'second_link_to_tool']
                joint_state.position = [self.prism,self.cont1,self.cont2]



                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                
                # Create new robot state
                self.manage_input()

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass
    def manage_input(self):
        up_key = self.get_parameter("up_key").get_parameter_value().string_value
        down_key = self.get_parameter("down_key").get_parameter_value().string_value
        first_left_key = self.get_parameter("first_left_key").get_parameter_value().string_value
        first_right_key = self.get_parameter("first_right_key").get_parameter_value().string_value
        second_left_key = self.get_parameter("second_left_key").get_parameter_value().string_value
        second_right_key = self.get_parameter("second_right_key").get_parameter_value().string_value
        with Input(keynames='curtsies') as input_generator:
            c = input_generator.send(0.001)
            if c == up_key:
                self.prism+=self.step
                if self.prism_limit(self.prism):
                    self.prism=0.
                self.start = self.iterations
            elif c == down_key:
                self.prism-=self.step
                if self.prism_limit(self.prism):
                    self.prism=-1.
                self.start = self.iterations
            elif c == first_left_key:
                self.cont1+=self.degree_step
                if self.cont_limit(self.cont1):
                    self.cont1=pi
                self.start = self.iterations
            elif c == first_right_key:
                self.cont1-=self.degree_step
                if self.cont_limit(self.cont1):
                    self.cont1=-pi
                self.start = self.iterations
            elif c == second_left_key:
                self.cont2+=self.degree_step
                if self.cont_limit(self.cont2):
                    self.cont2=pi
                self.start = self.iterations
            elif c == second_right_key:
                self.cont2-=self.degree_step
                if self.cont_limit(self.cont2):
                    self.cont2=-pi
                self.start = self.iterations
        self.iterations += 1

    def prism_limit(self,param):
    	return (self.prism_down_lim>=param or param>=self.prism_up_lim)
    	
    def cont_limit(self, param):
        return ( self.cont_down_lim>=param or param>= self.cont_up_lim)   

def main():
    node = StatePublisherTeleop()

if __name__ == '__main__':
    main()
