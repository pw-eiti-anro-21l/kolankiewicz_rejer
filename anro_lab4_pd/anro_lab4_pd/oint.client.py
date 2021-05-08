from lab4_interfaces.srv import Jintstructure 
import sys
import rclpy
from rclpy.node import Node


class OintClient(Node):

    def __init__(self):
        super().__init__('oint_client')
        self.cli = self.create_client(Jintstructure, 'jint_structure')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Jintstructure.Request()                                   # CHANGE

    def send_request(self):
        self.req.prismatic = float(sys.argv[1])
        self.req.angle1 = float(sys.argv[2])
        self.req.angle2 = float(sys.argv[3])
        self.req.time = float(sys.argv[4])  
        self.req.method = sys.argv[5]                # CHANGE
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    oint_client = OintClient()
    oint_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(oint_client)
        if oint_client.future.done():
            try:
                response = oint_client.future.result()
            except Exception as e:
                oint_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                oint_client.get_logger().info(
                    'Prismatic: %d, Continuous 1: %d,Continuous 2: %d,Time: %d,Method: %s, Response: %s' %                             
                    (oint_client.req.prismatic, oint_client.req.angle1,
                    oint_client.req.angle2,oint_client.req.time,
                    oint_client.req.method, response.resp))
                return 
    oint_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()