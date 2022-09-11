#!/usr/bin/python3
from franka_pendulum.msg import Sample
import rospy, rospkg
import sys

# Callback for receiving samples
def callback(sample):
    file.write(
        "pendulum_timestamp:                " + str(sample.pendulum_timestamp) + "\n" +
        "pendulum_angle:                    " + str(sample.pendulum_angle) + "\n" +
        "franka_timestamp:              " + str(sample.pendulum_angle) + "\n" +
        "franka_effector_position:      " + str(sample.franka_effector_position) + "\n" +
        "franka_effector_velocity:      " + str(sample.franka_effector_velocity) + "\n" +
        "command_timestamp:             " + str(sample.command_timestamp) + "\n" +
        "command_effector_acceleration: " + str(sample.command_effector_acceleration) + "\n\n"
    )

# Main
if __name__ == '__main__':
    # Read arguments
    namespace = "franka_pendulum"
    next_namespace = False
    log_path = rospkg.RosPack().get_path("franka_pendulum") + "/temp/log"
    next_log_path = False
    for i in sys.argv:
        if next_log_path: log_path = i
        elif next_namespace: namespace = i
        next_namespace = (i == "-N")
        next_log_path = (i == "-O")

    # Initialize
    rospy.init_node(namespace + "_logger")
    file = open(log_path, "w")
    sample_subscriber = rospy.Subscriber("/" + namespace + "/sample", Sample, callback)

    # Run
    rospy.spin()