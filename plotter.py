#!/usr/bin/python3
import rospy
import numpy as np
from franka_pole.msg import DebugSample
from matplotlib import pyplot as plt
from matplotlib import animation

class Plotter:
    # Sets current sample data
    def sample_callback(self, sample):
        self._pole_angle = sample.pole_angle
        self._pole_dangle = sample.pole_dangle
        self._franka_effector_y = sample.franka_effector_y
        self._franka_effector_dy = sample.franka_effector_dy
        self._desired_acceleration = sample.desired_acceleration

    # Samples data and plots it, called from Plot
    def animate_callback(self, i):
        # angle
        self._pole_angle_buffer = np.roll(self._pole_angle_buffer, -1)
        self._pole_angle_buffer[-1] = self._pole_angle
        self._pole_angle_line.set_data(self._time, self._pole_angle_buffer)
        # dangle
        self._pole_dangle_buffer = np.roll(self._pole_dangle_buffer, -1)
        self._pole_dangle_buffer[-1] = self._pole_dangle
        self._pole_dangle_line.set_data(self._time, self._pole_dangle_buffer)
        # effector_y
        self._franka_effector_y_buffer = np.roll(self._franka_effector_y_buffer, -1)
        self._franka_effector_y_buffer[-1] = self._franka_effector_y
        self._franka_effector_y_line.set_data(self._time, self._franka_effector_y_buffer)
        # efector_dy
        self._franka_effector_dy_buffer = np.roll(self._franka_effector_dy_buffer, -1)
        self._franka_effector_dy_buffer[-1] = self._franka_effector_dy
        self._franka_effector_dy_line.set_data(self._time, self._franka_effector_dy_buffer)
        # desired_acceleration
        self._desired_acceleration_buffer = np.roll(self._desired_acceleration_buffer, -1)
        self._desired_acceleration_buffer[-1] = self._desired_acceleration
        self._desired_acceleration_line.set_data(self._time, self._desired_acceleration_buffer)
        
        return self._pole_angle_line, self._pole_dangle_line, self._franka_effector_y_line, self._franka_effector_dy_line, self._desired_acceleration_line

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
        self._pole_angle_line, = self._axes[0].plot([], [], label="angle")
        self._pole_dangle_line, = self._axes[0].plot([], [], label="dangle")
        self._franka_effector_y_line, = self._axes[0].plot([], [], label="effector_y")
        self._franka_effector_dy_line, = self._axes[0].plot([], [], label="effector_dy")
        self._desired_acceleration_line, = self._axes[0].plot([], [], label="desired_acceleration")
        self._axes[0].legend(loc="upper left")

        # Creating buffers
        self._time = np.linspace(-self._sample_time, 0, self._sample_time * self._sample_frequency)
        self._pole_angle_buffer = np.zeros(self._sample_time * self._sample_frequency)
        self._pole_dangle_buffer = np.zeros(self._sample_time * self._sample_frequency)
        self._franka_effector_y_buffer = np.zeros(self._sample_time * self._sample_frequency)
        self._franka_effector_dy_buffer = np.zeros(self._sample_time * self._sample_frequency)
        self._desired_acceleration_buffer = np.zeros(self._sample_time * self._sample_frequency)

        self._pole_angle = 0
        self._pole_dangle = 0
        self._franka_effector_y = 0
        self._franka_effector_dy = 0
        self._desired_acceleration = 0

        # Starting subscriber
        self._sample_subscriber = rospy.Subscriber("/franka_pole/samples", DebugSample, lambda sample: self.sample_callback(sample))

        # Starting animation
        self._animation = animation.FuncAnimation(self._figure, lambda i: self.animate_callback(i), interval=1000/self._sample_frequency, blit=True)

if __name__ == '__main__':
    rospy.init_node('plotter')

    # Creating plotter
    plotter = Plotter()

    # Starting plotter
    plotter.spin()