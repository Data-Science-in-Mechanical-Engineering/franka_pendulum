#!/usr/bin/python3
from turtle import position
from franka_pole.msg import Sample, CommandParameters
import numpy as np
import rospy
import sys

# Callback for receiving samples
def callback(sample):
    global position, counter, duration
    counter += 1
    position += sample.franka_effector_position
    if counter >= duration:
        counter = 0
        position /= duration
        parameters.pole_angle_mean[0] += (position[0] - parameters.target_effector_position[0]) * parameters.pole_control[2] / parameters.pole_control[0]
        parameters.pole_angle_mean[1] += (position[1] - parameters.target_effector_position[1]) * parameters.pole_control[6] / parameters.pole_control[4]
        parameters_publisher.publish(parameters)
        position = np.zeros(3)

# Main
if __name__ == '__main__':
    # Read arguments
    namespace = "franka_pole"
    next_namespace = False
    for i in sys.argv:
        if next_namespace: namespace = i
        next_namespace = (i == "-N")

    # Initialize
    rospy.init_node(namespace + "_calibrate")
    sample_subscriber = rospy.Subscriber("/" + namespace + "/sample", Sample, callback)
    parameters_publisher = rospy.Publisher("/" + namespace + "/command_parameters", CommandParameters, queue_size=10)
    parameters = rospy.wait_for_message("/" + namespace + "/command_parameters", CommandParameters)
    position = np.zeros(3)
    counter = 0
    duration = 10*1000

    # Run
    rospy.spin()