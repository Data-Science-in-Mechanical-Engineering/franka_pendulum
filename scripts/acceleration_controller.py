#!/usr/bin/python3
import rospy
from franka_pole.msg import Sample, CommandAcceleration

class AccelerationController:
    def _sample_callback(self, sample):
        desired_acceleration = (
            -self._a * sample.pole_angle +
            -self._b * sample.ple_dangle +
            self._c * sample.franka_effector_y +
            self._d * sample.franka_effector_dy)

        command = CommandAcceleration()
        command.franka_effector_ddx = 0.0
        command.franka_effector_ddy = max(-0.6, min(sample.franka_effector_y + desired_acceleration, 0.6))
        command.franka_effector_ddz = 0.0
        self._command_publisher.publish(command)

    def __init__(self):
        rospy.init_node('acceleration_controller')

        self._a = 16.363880157470703 / 30
        self._b = 9.875003814697266 / 30
        self._c = 7.015979766845703 / 30
        self._d = 11.86760425567627 / 30

        self._optparams_subscriber = rospy.Subscriber('/franka_pole/sample', Sample, lambda sample: self._sample_callback(sample), queue_size=10)
        self._optparams_publisher = rospy.Publisher('/franka_pole/command_acceleration', CommandAcceleration, queue_size=10)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    controller = AccelerationController()
    controller.spin()