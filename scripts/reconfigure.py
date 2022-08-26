#!/usr/bin/python3
from franka_pole.msg import CommandParameters
import rospy
import numpy as np
import sys

# Main
if __name__ == '__main__':
    # Read arguments
    namespace = "franka_pole"
    next_namespace = False
    for i in sys.argv:
        if next_namespace: namespace = i
        next_namespace = (i == "-N")
    
    # Initialize
    rospy.init_node(namespace + "_reconfigure")
    parameters_publisher = rospy.Publisher("/" + namespace + "/command_parameters", CommandParameters, queue_size=10)
    parameters = rospy.wait_for_message("/" + namespace + "/command_parameters", CommandParameters)

    # Edit parameters
    parameters.target_effector_position = np.array(parameters.target_effector_position)
    if parameters.target_effector_position[2] > 0.6: parameters.target_effector_position[2] -= 0.1
    else: parameters.target_effector_position[2] += 0.1

    # Publish
    rospy.sleep(1.0)
    parameters_publisher.publish(parameters)