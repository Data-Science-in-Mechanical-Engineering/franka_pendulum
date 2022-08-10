#!/usr/bin/python3
from franka_pole.msg import Sample, CommandPosition, CommandParameters
import rospy
import sys

# Callback for receiving samples
def sample_callback(sample):
    command = CommandPosition()
    
    command.command_effector_position[0] = (
        control[0] * sample.pole_angle[1] +
        control[1] * sample.pole_dangle[1] +
        control[2] * (sample.franka_effector_position[0] - target[0]) +
        control[3] * sample.franka_effector_velocity[0])

    command.command_effector_position[1] = (
        control[4] * sample.pole_angle[0] +
        control[5] * sample.pole_dangle[0] +
        control[6] * (sample.franka_effector_position[1] - target[1]) +
        control[7] * sample.franka_effector_velocity[1])

    command.command_effector_position[2] = 0.0
    command.command_effector_velocity[0] = 0.0
    command.command_effector_velocity[1] = 0.0
    command.command_effector_velocity[2] = 0.0
    command_publisher.publish(command)

# Callback for receiving parameters
def parameters_callback(parameters):
    global target, control
    target = parameters.target_effector_position
    control = parameters.control

# Main
if __name__ == '__main__':
    # Read arguments
    namespace = "franka_pole"
    next_namespace = False
    for i in sys.argv:
        if next_namespace: namespace = i
        next_namespace = (i == "-N")

    # Initialize
    rospy.init_node(namespace + "_acceleration_controller")
    target = rospy.get_param("/" + namespace + "/target_effector_position")
    control = rospy.get_param("/" + namespace + "/control")
    command_publisher = rospy.Publisher("/" + namespace + "/command_position", CommandPosition, queue_size=10, tcp_nodelay=True)
    sample_subscriber = rospy.Subscriber("/" + namespace + "/sample", Sample, sample_callback, queue_size=10, tcp_nodelay=True)
    parameters_subscriber = rospy.Subscriber("/" + namespace + "/command_parameters", CommandParameters, parameters_callback, queue_size=10, tcp_nodelay=True)
    
    # Run
    rospy.spin()