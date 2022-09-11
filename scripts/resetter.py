#!/usr/bin/python3
from franka_pendulum.msg import CommandReset
import rospy
import sys

# Callback for timer
def callback(event):
    command = CommandReset()
    command.software = not hardware
    publisher.publish(command)

# Main
if __name__ == "__main__":
    # Read arguments
    namespace = "franka_pendulum"
    period = None
    hardware = False
    next_namespace = False
    next_period = False
    for i in sys.argv:
        if next_namespace: namespace = i
        elif next_period: period = float(i)
        next_namespace = (i == "-N")
        next_period = (i == "-T")
        if i == "-H": hardware = True

    # Initialize
    rospy.init_node(namespace + "_resetter")
    publisher = rospy.Publisher("/" + namespace + "/command_reset", CommandReset, queue_size=10)
    
    # Run
    if period is None:
        rospy.sleep(1.0)
        command = CommandReset()
        command.software = not hardware
        publisher.publish(command)
    
    else:
        timer = rospy.Timer(rospy.Duration(period), callback)
        rospy.spin()