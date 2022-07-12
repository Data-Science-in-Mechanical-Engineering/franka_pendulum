#!/usr/bin/python3
import rospy
from franka_pole.msg import Sample, CommandAcceleration

class AccelerationController:
    def _sample_callback(self, sample):
        if self._model == "2D" or self._model == "2Db": ddx_target = (
            self._a[0] * sample.pole_angle[1] +
            self._b[0] * sample.pole_dangle[1] +
            (self._c[0] * sample.franka_effector_position[0] - self._default[0]) +
            self._d[0] * sample.franka_effector_velocity[0])

        ddy_target = (
            self._a[1] * sample.pole_angle[0] +
            self._b[1] * sample.pole_dangle[0] +
            (self._c[1] * sample.franka_effector_position[1] - self._default[1]) +
            self._d[1] * sample.franka_effector_velocity[1])

        command = CommandAcceleration()
        command.command_effector_acceleration[0] = ddx_target if self._model == "2D" or self._model == "2Db" else 0.0
        command.command_effector_acceleration[1] = ddy_target
        command.command_effector_acceleration[2] = 0.0
        self._command_publisher.publish(command)

    def __init__(self):
        rospy.init_node('acceleration_controller')
        self._model = bool(rospy.get_param("/franka_pole/model"))
        self._min = rospy.get_param("/franka_pole/min_effector_position")
        self._max = rospy.get_param("/franka_pole/max_effector_position")
        self._default = rospy.get_param("/franka_pole/target_effector_position")

        if self._model == "1D":
            self._a = [ 0.0, 52.23421734 ]
            self._b = [ 0.0, 14.21253046 ]
            self._c = [ 0.0, 10.0 ]
            self._d = [ 0.0, 17.36721051 ]
        elif self._model == "2D":
            self._a = [ 6.69280164e+01, 5.16509316e+01 ]
            self._b = [ 1.94527670e+01, 1.42812800e+01 ]
            self._c = [ 1.41421356e+01, 1.00000000e+01 ]
            self._d = [ 2.50387242e+01, 1.74816219e+01 ]
        else:
            self._a = [ 7.38746335e+01, 5.63751219e+01 ]
            self._b = [ 1.84758010e+01, 1.38175352e+01 ]
            self._c = [ 1.00000000e+01, 7.07106781e+00 ]
            self._d = [ 1.64245608e+01, 1.17582709e+01 ]

        self._sample_subscriber = rospy.Subscriber('/franka_pole/sample', Sample, lambda sample: self._sample_callback(sample), queue_size=10)
        self._command_publisher = rospy.Publisher('/franka_pole/command_acceleration', CommandAcceleration, queue_size=10)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    controller = AccelerationController()
    controller.spin()