#!/usr/bin/python3
from franka_pole.msg import Sample, CommandParameters
import numpy as np
import rospy
import sys

# Callback for receiving samples
def callback(sample):
    global counter, duration, angle, position
    counter += 1
    angle += sample.pole_angle
    position += sample.franka_effector_position
    if counter >= duration:
        counter = 0
        angle /= duration
        position /= duration
        parameters.pole_angle_mean = np.array([
            parameters.pole_angle_mean[0] - angle[0],
            parameters.pole_angle_mean[1] - angle[1]
        ])
        print(parameters.pole_angle_mean)
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
    counter = 0
    duration = 10*100
    angle = np.zeros(2)
    position = np.zeros(3)
    sample_subscriber = rospy.Subscriber("/" + namespace + "/sample", Sample, callback)
    parameters_publisher = rospy.Publisher("/" + namespace + "/command_parameters", CommandParameters, queue_size=10)
    parameters = rospy.wait_for_message("/" + namespace + "/command_parameters", CommandParameters)

    # Run
    rospy.spin()