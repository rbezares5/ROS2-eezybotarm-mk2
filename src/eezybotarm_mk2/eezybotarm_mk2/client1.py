from interfaces.srv import TargetLocation      
import sys  # get access to command line input arguments
import rclpy
from rclpy.node import Node


class ClientAsync(Node):

    def __init__(self):
        super().__init__('coordinates_client_async')
        self.cli = self.create_client(TargetLocation, 'location_request')       
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TargetLocation.Request()                                   

    def send_request(self):
        self.req.x = float(sys.argv[1])
        self.req.y = float(sys.argv[2])
        self.req.z = float(sys.argv[3])                  
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    coordinates_client = ClientAsync()
    coordinates_client.send_request()

    # this loop checks if there is an available service with a matching name and type as the client
    while rclpy.ok():
        rclpy.spin_once(coordinates_client)
        if coordinates_client.future.done():
            try:
                response = coordinates_client.future.result()
            except Exception as e:
                coordinates_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                if response.goal == True:
                    coordinates_client.get_logger().info(
                        'Target location [%f,%f,%f] reached' %                              
                        (coordinates_client.req.x, coordinates_client.req.y, coordinates_client.req.z))
                break
            break

    coordinates_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()