# Import dependencies
import rclpy
from rclpy.node import Node

import sys  # get access to command line input arguments

# Import the custom message interface we created
from interfaces.msg import Location   

# The class Publisher is created, which inherits from Node
class Publisher(Node):

    def __init__(self):
        # the class constructor is called and gives name to the node
        super().__init__('Coordinates_publisher3')
        # the message to be sent is declared here
        self.publisher_ = self.create_publisher(Location, 'Arm_coord', 10) 


    # the message to be sent is built here
    def send_message(self):
        msg = Location()                 
        msg.x = float(sys.argv[1])
        msg.y = float(sys.argv[2])
        msg.z = float(sys.argv[3])
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing [x,y,z]: [%s,%s,%s]' % (msg.x,msg.y,msg.z))  

# The main function is defined, it initializes the library and then creates the node
def main(args=None):
    rclpy.init(args=args)

    coordinates_publisher3 = Publisher()
    
    coordinates_publisher3.send_message()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    coordinates_publisher3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()