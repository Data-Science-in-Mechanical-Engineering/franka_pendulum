#!/usr/bin/python3
import rospy
from franka_pole.msg import Sample, CommandPosition

class PositionController:
    def _sample_callback(self, sample):
        if self._two_dimensional: ddx_target = (
            self._a[0] * sample.pole_angle[1] +
            self._b[0] * sample.pole_joint_dangle[1] +
            (self._c[0] * sample.franka_effector[0] - self._default[0]) +
            self._d[0] * sample.franka_effector_velocity[0])

        ddy_target = (
            self._a[1] * sample.pole_angle[0] +
            self._b[1] * sample.pole_joint_dangle[0] +
            (self._c[1] * sample.franka_effector_position[1] - self._default[1]) +
            self._d[1] * sample.franka_effector_velocity[1])
        
        command = CommandPosition()
        command.command_effector_position[0] = sample.franka_effector_position[0] + ddx_target if self._two_dimensional else self._default[0]
        command.command_effector_position[1] = sample.franka_effector_position[1] + ddy_target
        command.command_effector_position[2] = self._default[2]
        for i in range(3):
            if command.command_effector_position[i] < self._min[i]: command.command_effector_position[i] = self._min[i]
            if command.command_effector_position[i] > self._max[i]: command.command_effector_position[i] = self._max[i]
        command.command_effector_velocity[0] = 0
        command.command_effector_velocity[1] = 0
        command.command_effector_velocity[2] = 0
        print(command.command_effector_position)
        self._command_publisher.publish(command)

    def __init__(self):
        rospy.init_node('position_controller')

        self._a = [ 16.363880157470703 / 30, 16.363880157470703 / 30 ]
        self._b = [ 9.875003814697266 / 30, 9.875003814697266 / 30 ]
        self._c = [ 7.015979766845703 / 30, 7.015979766845703 / 30 ]
        self._d = [ 11.86760425567627 / 30, 11.86760425567627 / 30 ]
        self._two_dimensional = bool(rospy.get_param("/franka_pole/two_dimensional"))
        self._min = rospy.get_param("/franka_pole/min_effector_position")
        self._max = rospy.get_param("/franka_pole/max_effector_position")
        self._default = rospy.get_param("/franka_pole/target_effector_position")

        self._sample_subscriber = rospy.Subscriber('/franka_pole/sample', Sample, lambda sample: self._sample_callback(sample), queue_size=10)
        self._command_publisher = rospy.Publisher('/franka_pole/command_position', CommandPosition, queue_size=10)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    controller = PositionController()
    controller.spin()