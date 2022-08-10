#!/usr/bin/python3
from franka_pole.msg import CommandReset
import rospy
import sys

# Callback for timer
def callback(event):
    command = CommandReset()
    command.software = True
    publisher.publish(command)

# Main
if __name__ == "__main__":
    # Read arguments
    namespace = "franka_pole"
    next_namespace = False
    for i in sys.argv:
        if next_namespace: namespace = i
        next_namespace = (i == "-N")

    # Initialize
    rospy.init_node(namespace + "_resetter")
    publisher = rospy.Publisher("/" + namespace + "/command_reset", CommandReset, queue_size=10)
    
    # Run
    timer = rospy.Timer(rospy.Duration(10), callback)
    rospy.spin()