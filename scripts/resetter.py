#!/usr/bin/python3
from franka_pole.msg import CommandReset
import rospy

if __name__ == "__main__":
    rospy.init_node('resetter')
    pub = rospy.Publisher("/franka_pole/command_reset", CommandReset, queue_size=10)
    commad = CommandReset()
    commad.software = True
    while True:
        rospy.sleep(10)
        pub.publish(commad)
