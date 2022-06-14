#!/usr/bin/python3
import rospy
from franka_pole.msg import Sample, CommandPosition

class PositionController:
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

        command = CommandPosition()
        command.franka_effector_x = max(0.25, min(sample.franka_effector_x + ddx_target, 0.75)) if self._two_dimensional else 0.5
        command.franka_effector_y = max(-0.6, min(sample.franka_effector_y + ddy_target, 0.6))
        command.franka_effector_z = 0.5
        command.franka_effector_dx = 0
        command.franka_effector_dy = 0
        command.franka_effector_dz = 0
        self._command_publisher.publish(command)

    def __init__(self):
        rospy.init_node('position_controller')

        self._a = 16.363880157470703 / 30
        self._b = 9.875003814697266 / 30
        self._c = 7.015979766845703 / 30
        self._d = 11.86760425567627 / 30
        self._two_dimensional = bool(rospy.get_param("~two_dimensional"))

        self._sample_subscriber = rospy.Subscriber('/franka_pole/sample', Sample, lambda sample: self._sample_callback(sample), queue_size=10)
        self._command_publisher = rospy.Publisher('/franka_pole/command_position', CommandPosition, queue_size=10)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    controller = PositionController()
    controller.spin()