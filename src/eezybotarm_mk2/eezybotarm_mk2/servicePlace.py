from interfaces.srv import TargetLocation 

import rclpy
from rclpy.node import Node

from .lib.kinematic_model import EEZYbotARM_Mk2
from .lib.serial_communication import arduinoController
from .lib.params import servoAngle_EE_closed, servoAngle_EE_open

class Service(Node):

    def __init__(self):
        super().__init__('coordinates_service')
        self.srv = self.create_service(TargetLocation, 'location_request', self.target_location_callback)        

    def target_location_callback(self, request, response):
        self.get_logger().info('Incoming location request\n [x,y,z] = [%f,%f,%f]' % (request.x, request.y, request.z)) 
        
        # Insert your Arduino serial port here to initialise the arduino controller
        myArduino = arduinoController(port="COM4")
        myArduino.openSerialPort()

        # Initialise kinematic model with initial joint angles (home position)
        myVirtualRobotArm = EEZYbotARM_Mk2(
            initial_q1=0, initial_q2=90, initial_q3=-130)

        # Assign new cartesian position where we want the robot arm end effector to move to
        # (x,y,z in mm from centre of robot base)
        x = request.x  # mm
        y = request.y  # mm
        z = request.z  # mm
        z1 = request.z +20  # mm


        # STEP 1. Position the tool above the target position and keep it closed
        # Compute inverse kinematics
        a1, a2, a3 = myVirtualRobotArm.inverseKinematics(x, y, z1)

        # Print the result
        #print('To move the end effector to the cartesian position (mm) x={}, y={}, z={}, the robot arm joint angles (degrees)  are q1 = {}, q2= {}, q3 = {}'.format(x, y, z, a1, a2, a3))

        # Visualise the new joint angles
        myVirtualRobotArm.updateJointAngles(q1=a1, q2=a2, q3=a3)
        #myVirtualRobotArm.plot()

        # Calculate the current servo angles
        servoAngle_q1, servoAngle_q2, servoAngle_q3 = myVirtualRobotArm.map_kinematicsToServoAngles()

        # Send the movement command to the arduino. The physical EEZYbotARM will move to this position
        myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=servoAngle_q1,
                                                            servoAngle_q2=servoAngle_q2,
                                                            servoAngle_q3=servoAngle_q3,
                                                            servoAngle_EE=servoAngle_EE_closed))

        # STEP 2. Reach target position
        # Compute inverse kinematics
        a1, a2, a3 = myVirtualRobotArm.inverseKinematics(x, y, z)
        # Update the new joint angles
        myVirtualRobotArm.updateJointAngles(q1=a1, q2=a2, q3=a3)
        # Calculate the current servo angles
        servoAngle_q1, servoAngle_q2, servoAngle_q3 = myVirtualRobotArm.map_kinematicsToServoAngles()
        # Send the movement command to the arduino. The physical EEZYbotARM will move to this position
        myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=servoAngle_q1,
                                                            servoAngle_q2=servoAngle_q2,
                                                            servoAngle_q3=servoAngle_q3,
                                                            servoAngle_EE=servoAngle_EE_closed))

        # STEP 3. Open tool to release the item
        # Send the movement command to the arduino. The physical EEZYbotARM will move to this position
        myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=servoAngle_q1,
                                                            servoAngle_q2=servoAngle_q2,
                                                            servoAngle_q3=servoAngle_q3,
                                                            servoAngle_EE=servoAngle_EE_open)) 

        # STEP 4. Lift the tool above target position after placing the item
        # Compute inverse kinematics
        a1, a2, a3 = myVirtualRobotArm.inverseKinematics(x, y, z1)
        # Update the new joint angles
        myVirtualRobotArm.updateJointAngles(q1=a1, q2=a2, q3=a3)
        # Calculate the current servo angles
        servoAngle_q1, servoAngle_q2, servoAngle_q3 = myVirtualRobotArm.map_kinematicsToServoAngles()
        # Send the movement command to the arduino. The physical EEZYbotARM will move to this position
        myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=servoAngle_q1,
                                                            servoAngle_q2=servoAngle_q2,
                                                            servoAngle_q3=servoAngle_q3,
                                                            servoAngle_EE=servoAngle_EE_closed))                                                                                                                                                                                     

        # Close the serial port
        myArduino.closeSerialPort()
        
        response.goal = True

        return response


def main(args=None):
    rclpy.init(args=args)

    coordinates_service = Service()

    rclpy.spin(coordinates_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()