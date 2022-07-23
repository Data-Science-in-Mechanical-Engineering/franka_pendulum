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
            lqr = [ 84.4717672, 19.32872174, 10.0, 15.88129816 ]
            self._a = [ 0.0, lqr[0] ]
            self._b = [ 0.0, lqr[1] ]
            self._c = [ 0.0, lqr[2] ]
            self._d = [ 0.0, lqr[3] ]
        elif self._model == "2D":
            lqr1 = [ 8.53149121e+01, 1.97783488e+01, 1.00000000e+01, 1.59353250e+01 ]
            lqr2 = [ 5.19597825e+01, 1.14967702e+01, 5.00000000e+00, 8.24415788e+00 ]
            self._a = [ lqr1[0], lqr2[0] ]
            self._b = [ lqr1[1], lqr2[1] ]
            self._c = [ lqr1[2], lqr2[2] ]
            self._d = [ lqr1[3], lqr2[3] ]
        else:
            lqr1 = [ 9.70119041e+01, 2.63787913e+01, 1.00000000e+01, 1.66667831e+01 ]
            lqr2 = [ 5.89905828e+01, 1.56870017e+01, 5.00000000e+00, 8.66793585e+00 ]
            self._a = [ lqr1[0], lqr2[0] ]
            self._b = [ lqr1[1], lqr2[1] ]
            self._c = [ lqr1[2], lqr2[2] ]
            self._d = [ lqr1[3], lqr2[3] ]

        self._sample_subscriber = rospy.Subscriber('/franka_pole/sample', Sample, lambda sample: self._sample_callback(sample), queue_size=10)
        self._command_publisher = rospy.Publisher('/franka_pole/command_acceleration', CommandAcceleration, queue_size=10)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    controller = AccelerationController()
    controller.spin()