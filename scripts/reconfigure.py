#!/usr/bin/python3
import rospy
import numpy as np
from franka_pole.msg import CommandParameters

if __name__ == '__main__':
    rospy.init_node('reconfigure')
    
    # Read
    parameters = rospy.wait_for_message("/franka_pole/command_parameters", CommandParameters)

    # Edit
    parameters.target_effector_position = np.array(parameters.target_effector_position)
    if parameters.target_effector_position[1] > 0: parameters.target_effector_position[1] = -0.1
    else: parameters.target_effector_position[1] = 0.1

    # Publish
    publisher = rospy.Publisher("/franka_pole/command_parameters", CommandParameters, queue_size=10)
    rospy.sleep(1.0)
    publisher.publish(parameters)