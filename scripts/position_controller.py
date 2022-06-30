#!/usr/bin/python3
import rospy
from franka_pole.msg import Sample, CommandPosition

class PositionController:
    def _sample_callback(self, sample):
        if self._two_dimensional: ddx_target = (
            self._a * sample.pole_angle[1] +
            self._b * sample.pole_joint_dangle[1] +
            self._c * sample.franka_effector[0] +
            self._d * sample.franka_effector_dx)

        ddy_target = (
            self._a * sample.pole_angle[0] +
            self._b * sample.pole_joint_dangle[0] +
            self._c * sample.franka_effector_position[1] +
            self._d * sample.franka_effector_velocity[1])

        command = CommandPosition()
        command.command_effector_position[0] = max(0.25, min(sample.franka_effector_position[0] + ddx_target, 0.75)) if self._two_dimensional else 0.5
        command.command_effector_position[1] = max(-0.6, min(sample.franka_effector_position[1] + ddy_target, 0.6))
        command.command_effector_position[2] = 0.5
        command.command_effector_velocity[0] = 0
        command.command_effector_velocity[1] = 0
        command.command_effector_velocity[2] = 0
        self._command_publisher.publish(command)

    def __init__(self):
        rospy.init_node('position_controller')

        self._a = 16.363880157470703 / 30
        self._b = 9.875003814697266 / 30
        self._c = 7.015979766845703 / 30
        self._d = 11.86760425567627 / 30
        self._two_dimensional = bool(rospy.get_param("/franka_pole/two_dimensional"))

        self._sample_subscriber = rospy.Subscriber('/franka_pole/sample', Sample, lambda sample: self._sample_callback(sample), queue_size=10)
        self._command_publisher = rospy.Publisher('/franka_pole/command_position', CommandPosition, queue_size=10)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    controller = PositionController()
    controller.spin()