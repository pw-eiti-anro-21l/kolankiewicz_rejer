from lab4_interfaces.srv import Jintstructure 
import sys
import rclpy
from rclpy.node import Node
import math


class JintClient(Node):

    def __init__(self):
        super().__init__('jint_client')
        self.cli = self.create_client(Jintstructure, 'jint_structure')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Jintstructure.Request()                                   # CHANGE

    def send_request(self):
        try:
            prism = float(sys.argv[1])
            ang1 = float(sys.argv[2])
            ang2 = float(sys.argv[3])
            tim = float(sys.argv[4])  
            met = sys.argv[5]
        except IndexError:
            print('Wrong number of parameters. Pismatic, angle1, angle2, time and method are expected.')
            raise Exception()
        try:
            if prism <-1 or prism >0:
                self.get_logger().info('Only values form range -1 to 0 are allowed for prismatic joint.')
                raise ValueError()
            elif ang1<-math.pi or ang1>math.pi:
                self.get_logger().info('Only values from range -pi to pi are allowed as angle1 parameter.')
                raise ValueError()
            elif ang2<-math.pi or ang2>math.pi:
                self.get_logger().info('Only values from range -pi to pi are allowed as angle2 parameter.')
                raise ValueError()
            elif tim ==0:
                self.get_logger().info('0 time is not allowed.')
                raise ValueError()
            elif met != "linear" and met!="polynomial":
                self.get_logger().info('There are only two specified methods: polynomial and linear.')
                raise ValueError()
            else:
                self.req.prismatic = prism
                self.req.angle1 = ang1
                self.req.angle2 = ang2
                self.req.time = tim 
                self.req.method = met
        except ValueError:
            print("Wrong parameters passed to client.")
            sys.exit(1)       
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    jint_client = JintClient()
    jint_client.send_request()
    while rclpy.ok():
        rclpy.spin_once(jint_client)
        if jint_client.future.done():
            try:
                response = jint_client.future.result()
            except Exception as e:
                jint_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                jint_client.get_logger().info(
                    'Prismatic: %d, Continuous 1: %d,Continuous 2: %d,Time: %d,Method: %s, Response: %s' %                             
                    (jint_client.req.prismatic, jint_client.req.angle1,
                    jint_client.req.angle2,jint_client.req.time,
                    jint_client.req.method, response.resp))
                return 
    jint_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()