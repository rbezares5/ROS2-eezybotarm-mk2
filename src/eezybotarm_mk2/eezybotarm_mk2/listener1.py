import rclpy
from rclpy.node import Node

from interfaces.msg import Location    

class Subscriber(Node):

    def __init__(self):
        super().__init__('coordinates_subscriber')
        self.subscription = self.create_subscription(
            Location,                                        
            'Arm_coord',                               
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Target [x,y,z] location received: [%s,%s,%s]' % (msg.x,msg.y,msg.z))     


def main(args=None):
    rclpy.init(args=args)

    coordinates_subscriber = Subscriber()

    rclpy.spin(coordinates_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    coordinates_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()