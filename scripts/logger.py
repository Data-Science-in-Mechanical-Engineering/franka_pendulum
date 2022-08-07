#!/usr/bin/python3
import rospy, rospkg
import sys
from franka_pole.msg import Sample

if __name__ == '__main__':
    # Get log name
    log_name = "log"
    next_log_name = False
    for i in sys.argv:
        if next_log_name: log_name = i
        next_log_name = (i == "-O")

    # Open log file
    rospy.init_node('logger')
    print(sys.argv)
    file = open(rospkg.RosPack().get_path("franka_pole") + "/temp/" + log_name, "w")

    # Write
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