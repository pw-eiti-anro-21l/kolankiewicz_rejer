from lab4_interfaces.srv import Ointstructure 
import sys
import rclpy
from rclpy.node import Node


class OintClient(Node):

    def __init__(self):
        super().__init__('oint_client')
        self.cli = self.create_client(Ointstructure, 'oint_structure')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Ointstructure.Request()                                   # CHANGE

    def send_request(self):
        self.req.x = float(sys.argv[1])
        self.req.y = float(sys.argv[2])
        self.req.z = float(sys.argv[3])
        self.req.roll = float(sys.argv[4])
        self.req.pitch = float(sys.argv[5])
        self.req.yaw = float(sys.argv[6])
        self.req.time = float(sys.argv[7])  
        self.req.method = sys.argv[8]                # CHANGE
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
                if str(oint_client.req.method) == "linear" or str(oint_client.req.method) == "polynomial":
                    oint_client.get_logger().info(
                        'x:%d, y:%d,z:%d, roll:%d, pitch:%d, yaw:%d, Time:%d, Method:%s, Response:%s' %                             
                        (oint_client.req.x, oint_client.req.y,
                        oint_client.req.z, oint_client.req.roll,
                        oint_client.req.pitch, oint_client.req.yaw,
                        oint_client.req.time,
                        oint_client.req.method, response.resp))
                else:
                    oint_client.get_logger().info('Response: Wrong method')
    oint_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()