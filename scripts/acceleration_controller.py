#!/usr/bin/python3
import rospy
from franka_pole.msg import Sample, CommandAcceleration

class AccelerationController:
    def _sample_callback(self, sample):
        if self._two_dimensional: ddx_target = (
            self._a * sample.pole_angle_y +
            self._b * sample.pole_dangle_y +
            self._c * sample.franka_effector_x +
            self._d * sample.franka_effector_dx)

        ddy_target = (
            self._a * sample.pole_angle_x +
            self._b * sample.pole_dangle_x +
            self._c * sample.franka_effector_y +
            self._d * sample.franka_effector_dy)

        command = CommandAcceleration()
        command.franka_effector_ddx = ddx_target if self._two_dimensional else 0.0
        command.franka_effector_ddy = ddy_target
        command.franka_effector_ddz = 0.0
        self._command_publisher.publish(command)

    def __init__(self):
        rospy.init_node('acceleration_controller')

        self._a = 16.363880157470703 * 10
        self._b = 9.875003814697266 * 10
        self._c = 7.015979766845703 * 10
        self._d = 11.86760425567627 * 10
        self._two_dimensional = bool(rospy.get_param("~two_dimensional"))

        self._sample_subscriber = rospy.Subscriber('/franka_pole/sample', Sample, lambda sample: self._sample_callback(sample), queue_size=10)
        self._command_publisher = rospy.Publisher('/franka_pole/command_acceleration', CommandAcceleration, queue_size=10)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    controller = AccelerationController()
    controller.spin()