#!/usr/bin/python3
import rospy, rospkg
import numpy as np
from franka_pole.msg import Sample

if __name__ == '__main__':
    rospy.init_node('logger')
    file = open(rospkg.RosPack().get_path("franka_pole") + "/temp/log", "w")

    def callback(sample):
        file.write(
            "pole_timestamp:                " + str(sample.pole_timestamp) + "\n" +
            "pole_angle:                    " + str(sample.pole_angle) + "\n" +
            "franka_timestamp:              " + str(sample.pole_angle) + "\n" +
            "franka_effector_position:      " + str(sample.franka_effector_position) + "\n" +
            "franka_effector_velocity:      " + str(sample.franka_effector_velocity) + "\n" +
            "command_timestamp:             " + str(sample.command_timestamp) + "\n" +
            "command_effector_acceleration: " + str(sample.command_effector_acceleration) + "\n\n"
        )
    
    sample_subscriber = rospy.Subscriber("/franka_pole/sample", Sample, callback)
    rospy.spin()