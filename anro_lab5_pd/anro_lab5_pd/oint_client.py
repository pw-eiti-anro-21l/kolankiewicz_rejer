from lab5_interfaces.srv import Ointstructure 
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
        try:
            x = float(sys.argv[1])
            z = float(sys.argv[2])
            time = float(sys.argv[3])  
            method = sys.argv[4]  
        except IndexError:
            print('Wrong number of parameters. x,z coordinates, time and method are expected.')
            raise Exception()
        try:
            if time ==0:
                self.get_logger().info('0 time is not allowed.')
                raise ValueError()
            elif method != "ellipse" and method!="rectangle":
                self.get_logger().info('There are only two specified methods: ellipse and rectangle.')
                raise ValueError()
            else:
                self.req.x = x
                self.req.z = z
                self.req.time = time  
                self.req.method = method  
        except ValueError:
            print("Wrong parameters passed to client.")
            sys.exit(1)       
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
                if str(oint_client.req.method) == "ellipse" or str(oint_client.req.method) == "rectangle":
                    oint_client.get_logger().info(
                        'x:%d,z:%d, Time:%d, Method:%s, Response:%s' %                             
                        (oint_client.req.x,
                        oint_client.req.z,
                        oint_client.req.time,
                        oint_client.req.method, response.resp))
                else:
                    oint_client.get_logger().info('Response: Wrong method')
                return
    oint_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
