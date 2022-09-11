#!/usr/bin/python3
from franka_pendulum.msg import Sample, CommandAcceleration, CommandParameters
import rospy
import sys

# Callback for receiving samples
def sample_callback(sample):
    command = CommandAcceleration()
    
    command.command_effector_acceleration[0] = (
        pendulum_control[0] * sample.pendulum_angle[1] +
        pendulum_control[1] * sample.pendulum_dangle[1] +
        pendulum_control[2] * (sample.franka_effector_position[0] - target[0]) +
        pendulum_control[3] * sample.franka_effector_velocity[0])

    command.command_effector_acceleration[1] = (
        pendulum_control[4] * sample.pendulum_angle[0] +
        pendulum_control[5] * sample.pendulum_dangle[0] +
        pendulum_control[6] * (sample.franka_effector_position[1] - target[1]) +
        pendulum_control[7] * sample.franka_effector_velocity[1])

    command.command_effector_acceleration[2] = 0.0
    command_publisher.publish(command)

# Callback for receiving parameters
def parameters_callback(parameters):
    global target, pendulum_control
    target = parameters.target_effector_position
    pendulum_control = parameters.pendulum_control

# Main
if __name__ == '__main__':
    # Read arguments
    namespace = "franka_pendulum"
    next_namespace = False
    for i in sys.argv:
        if next_namespace: namespace = i
        next_namespace = (i == "-N")

    # Initialize
    rospy.init_node(namespace + "_acceleration_controller")
    target = rospy.get_param("/" + namespace + "/target_effector_position")
    pendulum_control = rospy.get_param("/" + namespace + "/pendulum_control")
    command_publisher = rospy.Publisher("/" + namespace + "/command_acceleration", CommandAcceleration, queue_size=10, tcp_nodelay=True)
    sample_subscriber = rospy.Subscriber("/" + namespace + "/sample", Sample, sample_callback, queue_size=10, tcp_nodelay=True)
    parameters_subscriber = rospy.Subscriber("/" + namespace + "/command_parameters", CommandParameters, parameters_callback, queue_size=10, tcp_nodelay=True)
    
    # Run
    rospy.spin()