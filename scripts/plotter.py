#!/usr/bin/python3
import rospy
import numpy as np
from franka_pole.msg import Sample
from matplotlib import pyplot as plt
from matplotlib import animation

class Subscriber:
    """Listens all relevant topics"""

    def _sample_callback(self, sample):
        """Sets current sample
         - sample - received sample"""

        self.previous_filtered_effector_dx = self.filtered_effector_dx
        self.filtered_effector_dx = 0.95 * self.filtered_effector_dx + 0.05 * sample.franka_effector_velocity[0]
        self.previous_filtered_effector_dy = self.filtered_effector_dy
        self.filtered_effector_dy = 0.95 * self.filtered_effector_dy + 0.05 * sample.franka_effector_velocity[1]

        self.previous_franka_timestamp = self.sample.franka_timestamp
        self.sample = sample

    # Starts subscriber
    def __init__(self):
        """Initializes subscriber"""
        self.sample = Sample()
        self.filtered_effector_dx = 0.0
        self.filtered_effector_dy = 0.0
        self.previous_franka_timestamp = -np.Inf
        self.previous_filtered_effector_dx = 0.0
        self.previous_filtered_effector_dy = 0.0
        self.sample_subscriber = rospy.Subscriber("/franka_pole/sample", Sample, lambda sample: self._sample_callback(sample))

class Signal:
    """Represents one real value real-time signal"""

    def __init__(self, label, source):
        """Creates signal
         - label - label to be placed on the plot
         - source - function that returns float value"""
        self.label = label
        self.source = source

class Plot:
    """Represents real-time plot with time as X axis and signals as Y axis"""

    def __init__(self, title, line, column, signals, lower, upper, time):
        """Creates plot
         - title - title of the plot
         - line - vertical position on the window
         - column - horizontal position on the window
         - signals - list of signal objects
         - lower - lower bound of Y axis
         - upper - upper bound of Y axis
         - time - -time is lower bound of X axis, upper boind is zero"""
        self.title = title
        self.line = line
        self.column = column
        self.signals = signals
        self.lower = lower
        self.upper = upper
        self.time = time

        self._frequency = None
        self._axes = None           # matplotlib axes object, one for plot
        self._time_buffer = None    # numpy buffer for time, one for plot
        self._lines = None          # matplotlib line object, one for plot and signal
        self._buffers = None        # numpy buffer, one for plot and signal

    def _init(self, frequency):
        """Fully initializes plot, internal use only"""
        # Create buffers
        self._frequency = frequency
        self._time_buffer = np.linspace(-self.time, 0, self.time * frequency)
        self._lines = [ None for signal in self.signals ]
        self._buffers = [ np.zeros(self.time * frequency) for signal in self.signals ]

        # Set limits
        self._axes.set_xlim([-self.time, 0])
        self._axes.set_ylim([self.lower, self.upper])
        self._axes.set_title(self.title)

        # Create lines
        for i in range(len(self.signals)): self._lines[i], = self._axes.plot([], [], label = self.signals[i].label)

        # Set legend position
        self._axes.legend(loc = "upper left")

    def _animate(self):
        """Returns lines, internal use only"""
        for i in range(len(self.signals)):
            self._buffers[i] = np.roll(self._buffers[i], -1)
            self._buffers[i][-1] = self.signals[i].source()
            self._lines[i].set_data(self._time_buffer, self._buffers[i])
        return self._lines

class BarChart:
    """Represents real-time bar charts with multiple bars and signals as their heights"""

    def __init__(self, title, line, column, signals, lower, upper):
        """Represents bar chart
         - title - title of the bar chart
         - line - vertical position on the window
         - column - horizontal position on the window
         - signals - list of signal objects
         - lower - lower bound of Y axis
         - upper - upper bound of Y axis"""
        self.title = title
        self.line = line
        self.column = column
        self.signals = signals
        self.lower = lower
        self.upper = upper

        self._frequency = None
        self._axes = None       # matplotlib axes object, one for plot
        self._bars = None       # matplotlib bar container object, one for plot

    def _init(self, frequency):
        """Fully initializes bar chart, internal use only"""
        self._frequency = frequency

        # Set limits
        self._axes.set_ylim([self.lower, self.upper])
        self._axes.set_title(self.title)

        # Create bars
        self._bars = self._axes.bar([ signal.label for signal in self.signals ], np.zeros(len(self.signals)))

    def _animate(self):
        """Returns bars, internal use only"""
        for i in range(len(self.signals)): self._bars[i].set_height(self.signals[i].source())
        return self._bars

class Plotter:
    """Plotter, plots data with given frequency"""
    
    def _animate_callback(self):
        """Shifts buffers and returns lines, internal use only"""
        objects = []
        for plot in self.plots: objects.extend(plot._animate())
        return objects

    def __init__(self, caption, plots, frequency):
        """Creates plotter
         - caption - caption of the window
         - plots - list of plots
         - frequency - plotting frequency"""
        self.caption = caption
        self.plots = plots

        # Check plots
        max_line = 0
        max_column = 0
        for plot in plots:
            if plot.line < 0: raise Exception("Line less than zero in " + plot.title)
            if plot.column < 0: raise Exception("Line column than zero in " + plot.title)
            if plot.line > max_line: max_line = plot.line
            if plot.column > max_column: max_column = plot.column

        plot_matrix = [[None for column in range(max_column+1)] for line in range(max_line+1)]
        for plot in plots:
            if plot_matrix[plot.line][plot.column] != None: raise Exception("Several plots on same position: " + plot_matrix[plot.line][plot.column].title + ", " + plot.title)
            else: plot_matrix[plot.line][plot.column] = plot

        for line in range(max_line+1):
            for column in range(max_column+1):
                if plot_matrix[line][column] == None: raise Exception("No plot on position: " + str(line) + ", " + str(column))

        # Create figure and axes
        self.figure, axes = plt.subplots(max_line+1, max_column+1, num = caption)
        if max_line == 0 and max_column == 0:
            plots[0]._axes = axes
        elif max_line != 0 and max_column == 0:
            for i in range(len(plots)): plots[i]._axes = axes[plots[i].line]
        elif max_line == 0 and max_column != 0:
            for i in range(len(plots)): plots[i]._axes = axes[plots[i].column]
        else:
            for i in range(len(plots)): plots[i]._axes = axes[plots[i].line, plots[i].column]

        # Fully initialize plots
        for plot in plots: plot._init(frequency)

        # Create animation
        self.animation = animation.FuncAnimation(self.figure, lambda i: self._animate_callback(), interval=1000/frequency, blit=True)

    def start(self):
        """Runs plotter"""
        plt.show()

if __name__ == '__main__':
    rospy.init_node('plotter')
    
    subscriber = Subscriber()
    
    pole_angle_x = Signal("Pole angle around X", lambda: subscriber.sample.pole_angle[0])
    pole_angle_y = Signal("Pole angle around Y", lambda: subscriber.sample.pole_angle[1])
    pole_dangle_x = Signal("Pole rotation around X", lambda: subscriber.sample.pole_dangle[0])
    pole_dangle_y = Signal("Pole rotation around Y", lambda: subscriber.sample.pole_dangle[1])
    franka_effector_x = Signal("Effector X", lambda: subscriber.sample.franka_effector_position[0])
    franka_effector_y = Signal("Effector Y", lambda: subscriber.sample.franka_effector_position[1])
    franka_effector_dx = Signal("Effector X velocity", lambda: subscriber.sample.franka_effector_velocity[0])
    franka_effector_dy = Signal("Effector Y velocity", lambda: subscriber.sample.franka_effector_velocity[1])
    franka_effector_ddx = Signal("Effector X acceleration", lambda: (subscriber.filtered_effector_dx - subscriber.previous_filtered_effector_dx) / (subscriber.sample.franka_timestamp - subscriber.previous_franka_timestamp))
    franka_effector_ddy = Signal("Effector Y acceleration", lambda: (subscriber.filtered_effector_dy - subscriber.previous_filtered_effector_dy) / (subscriber.sample.franka_timestamp - subscriber.previous_franka_timestamp))
    command_effector_ddx = Signal("Desired X acceleration", lambda: subscriber.sample.command_effector_acceleration[0])
    command_effector_ddy = Signal("Desired Y acceleration", lambda: subscriber.sample.command_effector_acceleration[1])
    
    position_plot_x = Plot("Position X", 0, 0, [ pole_angle_y, franka_effector_x ], -1, 1, 3)
    position_plot_y = Plot("Position Y", 0, 1, [ pole_angle_x, franka_effector_y ], -1, 1, 3)
    acceleration_plot_x = Plot("Acceleration X", 1, 0, [ franka_effector_ddx, command_effector_ddx ], -50, 50, 3)
    acceleration_plot_y = Plot("Acceleration Y", 1, 1, [ franka_effector_ddy, command_effector_ddy ], -50, 50, 3)
    
    plotter = Plotter("Plotter", [ position_plot_x, position_plot_y, acceleration_plot_x, acceleration_plot_y ], 50)
    
    plotter.start()