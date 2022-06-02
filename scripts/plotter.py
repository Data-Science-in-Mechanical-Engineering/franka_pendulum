#!/usr/bin/python3
import rospy
import numpy as np
from franka_pole.msg import Sample
from matplotlib import pyplot as plt
from matplotlib import animation

sample_frequency = 50

# Listens all relevant topics
class Subscriber:
    # Sets current sample data
    def _sample_callback(self, sample):
        # Shift franka variables
        if sample.franka_timestamp != self.previous_sample.franka_timestamp:
            self.previous_sample.franka_timestamp = self.sample.franka_timestamp
            self.previous_sample.franka_effector_y = self.sample.franka_effector_y
            self.previous_sample.franka_effector_dy = self.sample.franka_effector_dy

        # Getting sample
        self.sample = sample


    # Starts subscriber
    def __init__(self):
        self.sample = Sample()
        self.previous_sample = Sample()
        self.previous_sample.franka_timestamp = -np.Inf
        self.previous_sample.pole_timestamp = -np.Inf
        self.previous_sample.control_timestamp = -np.Inf
        self.sample_subscriber = rospy.Subscriber("/franka_pole/sample", Sample, lambda sample: self._sample_callback(sample))

class Signal:
    # Creates signal
    # label - Label on the plot
    # source - Function that returns float value
    def __init__(self, label, source, filter_factor = 0.0):
        self.label = label
        self.source = source
        self.filter_factor = filter_factor
        self.value = 0.0

class Plot:
    # Creates plot
    # title - Title of the plot
    # line - Vertical position on screen
    # column - Horizontal position on screen
    # signals - List of Signal objects
    # lower - Lower limit of the plot
    # upper - Upper limit of the plot
    # time - Lower limit of the plot in time axis
    def __init__(self, title, line, column, signals, lower, upper, time):
        self.title = title
        self.line = line
        self.column = column
        self.signals = signals
        self.lower = lower
        self.upper = upper
        self.time = time

        self.axes = None                                                    # matplotlib axes object, one for plot
        self.time_buffer = np.linspace(-time, 0, time * sample_frequency)   # numpy buffer for time, one for plot
        self.lines = []                                                     # matplotlib line object, one for plot and signal
        self.buffers = []                                                   # numpy buffer, one for plot and signal
        for signal in signals:
            self.lines.append(None)
            self.buffers.append(np.zeros(self.time * sample_frequency))

class BarChart:
    # Creates bar chart
    # title - Title of the bar chart
    # line - Vertical position on screen
    # column - Horizontal position on screen
    # signals - List of Signal objects
    # lower - Lower limit of the plot
    # upper - Upper limit of the plot
    def __init__(self, title, line, column, signals, lower, upper):
        self.title = title
        self.line = line
        self.column = column
        self.signals = signals
        self.lower = lower
        self.upper = upper

        self.axes = None    # matplotlib axes object, one for plot
        self.bars = None    # matplotlib bar container object, one for plot

# Pltos data with given frequency
class Plotter:
    # Shifts buffers and returns lines
    def _animate_callback(self):
        objects = []
        for plot in self.plots:
            if type(plot) is BarChart:
                for i in range(len(plot.signals)):
                    plot.signals[i].value = plot.signals[i].filter_factor * plot.signals[i].value + (1.0 - plot.signals[i].filter_factor) * plot.signals[i].source()
                    plot.bars[i].set_height(plot.signals[i].value)
                    objects.append(plot.bars[i])
            else:
                for i in range(len(plot.signals)):
                    plot.buffers[i] = np.roll(plot.buffers[i], -1)
                    plot.signals[i].value = plot.signals[i].filter_factor * plot.signals[i].value + (1.0 - plot.signals[i].filter_factor) * plot.signals[i].source()
                    plot.buffers[i][-1] = plot.signals[i].value
                    plot.lines[i].set_data(plot.time_buffer, plot.buffers[i])
                    objects.append(plot.lines[i])
        return objects

    # Creates Plotter
    def __init__(self, caption, plots):
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
        if len(plots) == 1: plots[0].axes = axes
        else:
            for i in range(len(plots)): plots[i].axes = axes[i]

        # Set limits and titles
        for plot in plots:
            if not type(plot) is BarChart: plot.axes.set_xlim([-plot.time, 0])
            plot.axes.set_ylim([plot.lower, plot.upper])
            plot.axes.set_title(plot.title)

        # Create lines/bars
        for plot in plots:
            if type(plot) is BarChart:
                labels = []
                for i in range(len(plot.signals)):
                    labels.append(plot.signals[i].label)
                plot.bars = plot.axes.bar(labels, np.ones(len(plot.signals)))
            else:
                for i in range(len(plot.signals)):
                    plot.lines[i], = plot.axes.plot([], [], label = plot.signals[i].label)

        # Set legend positions
        for plot in plots:
            if not type(plot) is BarChart: plot.axes.legend(loc = "upper left")

        self.animation = animation.FuncAnimation(self.figure, lambda i: self._animate_callback(), interval=1000/sample_frequency, blit=True)

    # Runs plotter
    def start(self):        
        plt.show()


if __name__ == '__main__':
    rospy.init_node('plotter')
    
    subscriber = Subscriber()
    
    pole_angle = Signal("Pole angle", lambda: subscriber.sample.pole_angle)
    pole_dangle = Signal("Pole rotaion velocity", lambda: subscriber.sample.pole_dangle)
    franka_effector_y = Signal("Effector Y", lambda: subscriber.sample.franka_effector_y)
    franka_effector_dy = Signal("Effector Y velocity", lambda: subscriber.sample.franka_effector_dy)
    franka_effector_ddy = Signal("Effector Y acceleration", lambda: (subscriber.sample.franka_effector_dy - subscriber.previous_sample.franka_effector_dy) / (subscriber.sample.franka_timestamp - subscriber.previous_sample.franka_timestamp), 0.8)
    control_effector_ddy = Signal("Desired Y acceleration", lambda: subscriber.sample.control_effector_ddy)
    
    position_plot = Plot("Position", 0, 0, [ pole_angle, franka_effector_y ], -1, 1, 3)
    velocity_plot = Plot("Velocity", 0, 1, [ pole_dangle, franka_effector_dy ], -5, 5, 3)
    acceleration_plot = Plot("Acceleration", 0, 2, [ franka_effector_ddy, control_effector_ddy ], -50, 50, 3)
    
    plotter = Plotter("Plotter", [ position_plot, velocity_plot, acceleration_plot ])
    
    plotter.start()