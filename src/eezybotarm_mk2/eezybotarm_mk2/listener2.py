import rclpy
from rclpy.node import Node

from interfaces.msg import Location    

from .lib.kinematic_model import EEZYbotARM_Mk2
from .lib.serial_communication import arduinoController

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
        
        # Insert your Arduino serial port here to initialise the arduino controller
        myArduino = arduinoController(port="COM4")
        myArduino.openSerialPort()

        # Initialise kinematic model with initial joint angles (home position)
        myVirtualRobotArm = EEZYbotARM_Mk2(
            initial_q1=0, initial_q2=90, initial_q3=-130)
        # Plot it
        myVirtualRobotArm.plot()

        # Define end effector open and closed angle
        servoAngle_EE_closed = 10
        servoAngle_EE_open = 90

        # Calculate the current servo angles
        servoAngle_q1, servoAngle_q2, servoAngle_q3 = myVirtualRobotArm.map_kinematicsToServoAngles()

        # Send the movement command to the arduino. The physical EEZYbotARM will move to this position
        myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=servoAngle_q1,
                                                            servoAngle_q2=servoAngle_q2,
                                                            servoAngle_q3=servoAngle_q3,
                                                            servoAngle_EE=servoAngle_EE_open))


        # Assign new cartesian position where we want the robot arm end effector to move to
        # (x,y,z in mm from centre of robot base)
        x = msg.x  # mm
        y = msg.y  # mm
        z = msg.z  # mm

        # Compute inverse kinematics
        a1, a2, a3 = myVirtualRobotArm.inverseKinematics(x, y, z)

        # Print the result
        print('To move the end effector to the cartesian position (mm) x={}, y={}, z={}, the robot arm joint angles (degrees)  are q1 = {}, q2= {}, q3 = {}'.format(x, y, z, a1, a2, a3))

        # Visualise the new joint angles
        myVirtualRobotArm.updateJointAngles(q1=a1, q2=a2, q3=a3)
        myVirtualRobotArm.plot()

        # Calculate the current servo angles
        servoAngle_q1, servoAngle_q2, servoAngle_q3 = myVirtualRobotArm.map_kinematicsToServoAngles()

        # Send the movement command to the arduino. The physical EEZYbotARM will move to this position
        myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=servoAngle_q1,
                                                            servoAngle_q2=servoAngle_q2,
                                                            servoAngle_q3=servoAngle_q3,
                                                            servoAngle_EE=servoAngle_EE_open))

        # Close the serial port
        myArduino.closeSerialPort()



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