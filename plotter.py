#!/usr/bin/python3
import rospy
import numpy as np
from franka_pole.msg import DebugSample
from matplotlib import pyplot as plt
from matplotlib import animation

class Plotter:
    # Sets current sample data
    def sample_callback(self, sample):
        self._angle = sample.angle
        self._dangle = sample.dangle
        self._effector_y = sample.effector_y
        self._effector_dy = sample.effector_dy
        self._desired_acceleration = sample.desired_acceleration

    # Samples data and plots it, called from Plot
    def animate_callback(self, i):
        # angle
        self._angle_buffer = np.roll(self._angle_buffer, -1)
        self._angle_buffer[-1] = self._angle
        self._angle_line.set_data(self._time, self._angle_buffer)
        # dangle
        self._dangle_buffer = np.roll(self._dangle_buffer, -1)
        self._dangle_buffer[-1] = self._dangle
        self._dangle_line.set_data(self._time, self._dangle_buffer)
        # effector_y
        self._effector_y_buffer = np.roll(self._effector_y_buffer, -1)
        self._effector_y_buffer[-1] = self._effector_y
        self._effector_y_line.set_data(self._time, self._effector_y_buffer)
        # efector_dy
        self._effector_dy_buffer = np.roll(self._effector_dy_buffer, -1)
        self._effector_dy_buffer[-1] = self._effector_dy
        self._effector_dy_line.set_data(self._time, self._effector_dy_buffer)
        # desired_acceleration
        self._desired_acceleration_buffer = np.roll(self._desired_acceleration_buffer, -1)
        self._desired_acceleration_buffer[-1] = self._desired_acceleration
        self._desired_acceleration_line.set_data(self._time, self._desired_acceleration_buffer)
        
        return self._angle_line, self._dangle_line, self._effector_y_line, self._effector_dy_line, self._desired_acceleration_line

    # Waits for termination
    def spin(self):
        plt.show()

    # Creates Plot
    def __init__(self):
        # Configuretion
        self._sample_time = 10
        self._sample_frequency = 20

        # Creating lines
        self._figure, self._axes = plt.subplots(1, 1, num="franka_pole")
        self._axes = [ self._axes ] # If one subplot
        self._axes[0].set_xlim([-self._sample_time, 0])
        self._axes[0].set_ylim([-10,10])
        #self._axes[0].set_title("title")
        self._angle_line, = self._axes[0].plot([], [], label="angle")
        self._dangle_line, = self._axes[0].plot([], [], label="dangle")
        self._effector_y_line, = self._axes[0].plot([], [], label="effector_y")
        self._effector_dy_line, = self._axes[0].plot([], [], label="effector_dy")
        self._desired_acceleration_line, = self._axes[0].plot([], [], label="desired_acceleration")
        self._axes[0].legend(loc="upper left")

        # Creating buffers
        self._time = np.linspace(-self._sample_time, 0, self._sample_time * self._sample_frequency)
        self._angle_buffer = np.zeros(self._sample_time * self._sample_frequency)
        self._dangle_buffer = np.zeros(self._sample_time * self._sample_frequency)
        self._effector_y_buffer = np.zeros(self._sample_time * self._sample_frequency)
        self._effector_dy_buffer = np.zeros(self._sample_time * self._sample_frequency)
        self._desired_acceleration_buffer = np.zeros(self._sample_time * self._sample_frequency)

        self._angle = 0
        self._dangle = 0
        self._effector_y = 0
        self._effector_dy = 0
        self._desired_acceleration = 0

        # Starting subscriber
        self._sample_subscriber = rospy.Subscriber("/franka_pole/debug_samples", DebugSample, lambda sample: self.sample_callback(sample))

        # Starting animation
        self._animation = animation.FuncAnimation(self._figure, lambda i: self.animate_callback(i), interval=1000/self._sample_frequency, blit=True)

if __name__ == '__main__':
    rospy.init_node('plotter')

    # Creating plotter
    plotter = Plotter()

    # Starting plotter
    plotter.spin()