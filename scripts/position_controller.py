#!/usr/bin/python3
import rospy
from franka_pole.msg import Sample, CommandPosition, CommandParameters

class PositionController:
    def _sample_callback(self, sample):
        command = CommandPosition()
        
        command.command_effector_position[0] = (
            self._control[0] * sample.pole_angle[1] +
            self._control[1] * sample.pole_dangle[1] +
            self._control[2] * (sample.franka_effector_position[0] - self._target[0]) +
            self._control[3] * sample.franka_effector_velocity[0])

        command.command_effector_position[1] = (
            self._control[4] * sample.pole_angle[0] +
            self._control[5] * sample.pole_dangle[0] +
            self._control[6] * (sample.franka_effector_position[1] - self._target[1]) +
            self._control[7] * sample.franka_effector_velocity[1])

        command.command_effector_position[2] = 0.0
        command.command_effector_velocity[0] = 0.0
        command.command_effector_velocity[1] = 0.0
        command.command_effector_velocity[2] = 0.0
        self._command_publisher.publish(command)

    def _parameters_callback(self, parameters):
        self._taregt = parameters.target_effector_position
        self._control = parameters.control

    def __init__(self):
        rospy.init_node('position_controller')
        
        self._target = rospy.get_param("/franka_pole/target_effector_position")
        self._control = rospy.get_param("/franka_pole/control")

        self._sample_subscriber = rospy.Subscriber('/franka_pole/sample', Sample, lambda sample: self._sample_callback(sample), queue_size=10, tcp_nodelay=True)
        self._parameters_subscriber = rospy.Subscriber('/franka_pole/command_parameters', CommandParameters, lambda sample: self._parameters_callback(sample), queue_size=10, tcp_nodelay=True)
        self._command_publisher = rospy.Publisher('/franka_pole/command_position', CommandPosition, queue_size=10, tcp_nodelay=True)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    controller = PositionController()
    controller.spin()