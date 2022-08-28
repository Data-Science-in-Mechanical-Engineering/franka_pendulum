#!/usr/bin/python3
import rospy, rosbag, rospkg
import numpy as np
import sys
from franka_pole.msg import Sample
from matplotlib import pyplot as plt
from matplotlib import animation
from threading import Lock

class Signal:
    """Represents one real value real-time signal"""

    def __init__(self, label):
        """Creates signal
         - label - label to be placed on the plot
         - source - function that returns float value"""
        self.label = label
        self._timestamp = -np.Inf
        self._timestamps = np.zeros(0)
        self._values = np.zeros(0)
        self._lock = Lock()

        self._frequency = None

    def _init(self, frequency):
        """Fully initializes signal, internal use only"""
        self._frequency = frequency

    def set(self, timestamp, value):
        """Sets new value to a signal
         - timestamp - time of the measurement
         - value - new value"""
        self._lock.acquire()
        if timestamp - self._timestamp > 1.0/self._frequency:
            self._timestamp = timestamp
            self._timestamps = np.append(self._timestamps, timestamp)
            self._values = np.append(self._values, value)
        self._lock.release()

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
        self._lines = None          # matplotlib line object, one for plot/signal
        self._buffers = None        # numpy buffer, one for plot/signal
        self._time_buffers = None   # numpy buffer for time, one for plot/signal
        self._axes = None           # matplotlib axes object, one for plot

    def _init(self, frequency):
        """Fully initializes plot, internal use only"""
        self._frequency = frequency
        for signal in self.signals: signal._init(frequency)

        # Create buffers
        self._buffers = [ np.zeros(0) for signal in self.signals ]
        self._time_buffers = [ np.zeros(0) for signal in self.signals ]

        # Set limits
        self._axes.set_xlim([-self.time, 0])
        self._axes.set_ylim([self.lower, self.upper])
        self._axes.set_title(self.title)

        # Create lines
        self._lines = [ None for signal in self.signals ]
        for i in range(len(self.signals)): self._lines[i], = self._axes.plot([], [], label = self.signals[i].label)

        # Set legend position
        self._axes.legend(loc = "upper left")

    def _animate(self):
        """Returns lines, internal use only"""
        latest = -np.Inf
        for signal in self.signals:
            if signal._timestamp > latest: latest = signal._timestamp

        for i in range(len(self.signals)):
            self.signals[i]._lock.acquire() #Multithreading issues in Python, who could have thought?
            existing = len(self._time_buffers[i])
            new = len(self.signals[i]._timestamps)
            limit = self.time * self._frequency
            if new != 0:
                if new > limit:
                    self._time_buffers[i] = self._time_buffers[i][-limit:]
                    self._buffers[i] = self._buffers[i][-limit:]
                elif new + existing > limit:
                    self._time_buffers[i] = np.concatenate((self._time_buffers[i][new+existing-limit:], self.signals[i]._timestamps))
                    self._buffers[i] = np.concatenate((self._buffers[i][new+existing-limit:], self.signals[i]._values))
                else:
                    self._time_buffers[i] = np.concatenate((self._time_buffers[i], self.signals[i]._timestamps))
                    self._buffers[i] = np.concatenate((self._buffers[i], self.signals[i]._values))
                self._lines[i].set_data(self._time_buffers[i] - latest, self._buffers[i])
                self.signals[i]._timestamps = np.zeros(0)
                self.signals[i]._values = np.zeros(0)
            self.signals[i]._lock.release()

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
        for signal in self.signals: signal._init(frequency)

        # Set limits
        self._axes.set_ylim([self.lower, self.upper])
        self._axes.set_title(self.title)

        # Create bars
        self._bars = self._axes.bar([ signal.label for signal in self.signals ], np.zeros(len(self.signals)))

    def _animate(self):
        """Returns bars, internal use only"""
        for i in range(len(self.signals)):
            self.signals[i]._lock.acquire()
            if len(self.signals[i]._timestamps) != 0:
                self._bars[i].set_height(self.signals[i]._values[-1])
                self.signals[i]._timestamps = np.zeros(0)
                self.signals[i]._values = np.zeros(0)
            self.signals[i]._lock.release()
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
        self.frequency = frequency

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
        self._figure, axes = plt.subplots(max_line+1, max_column+1, num = caption)
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

    def start(self):
        """Runs plotter"""
        self.animation = animation.FuncAnimation(self._figure, lambda i: self._animate_callback(), interval=1000/self.frequency, blit=True)
        plt.show()

# Main
if __name__ == '__main__':
    # Get aruments
    log_name = "log.bag"
    namespace = "franka_pole"
    start_time = None
    next_log_name = False
    next_start_time = False
    next_namespace = False
    for i in sys.argv:
        if next_log_name: log_name = i
        elif next_namespace: namespace = i
        elif next_start_time: start_time = float(i)
        next_log_name = (i == "-I")
        next_namespace = (i == "-N")
        next_start_time = (i == "-S")

    # Plot structure
    pole_angle_x = Signal("Pole angle around X")
    pole_angle_y = Signal("Pole angle around Y")
    franka_effector_x = Signal("Effector X")
    franka_effector_y = Signal("Effector Y")
    franka_effector_ddx = Signal("Effector X acceleration")
    franka_effector_ddy = Signal("Effector Y acceleration")
    command_effector_ddx = Signal("Desired X acceleration")
    command_effector_ddy = Signal("Desired Y acceleration")
    
    position_plot_x = Plot("Position X", 0, 0, [ pole_angle_y, franka_effector_x ], -1, 1, 3)
    position_plot_y = Plot("Position Y", 0, 1, [ pole_angle_x, franka_effector_y ], -1, 1, 3)
    acceleration_plot_x = Plot("Acceleration X", 1, 0, [ franka_effector_ddx, command_effector_ddx ], -10, 10, 3)
    acceleration_plot_y = Plot("Acceleration Y", 1, 1, [ franka_effector_ddy, command_effector_ddy ], -10, 10, 3)
    
    plotter = Plotter(namespace, [ position_plot_x, position_plot_y, acceleration_plot_x, acceleration_plot_y ], 200)

    # Message processing
    previous_franka_timestamp = -np.Inf
    filtered_effector_dx = 0.0
    filtered_effector_dy = 0.0
    previous_filtered_effector_dx = 0.0
    previous_filtered_effector_dy = 0.0
    filter_factor = 0.5

    def callback(sample):
        global previous_franka_timestamp, filtered_effector_dx, filtered_effector_dy, previous_filtered_effector_dx, previous_filtered_effector_dy
        
        filtered_effector_dx = (1.0 - filter_factor) * filtered_effector_dx + filter_factor * sample.franka_effector_velocity[0]
        filtered_effector_dy = (1.0 - filter_factor) * filtered_effector_dy + filter_factor * sample.franka_effector_velocity[1]

        pole_angle_x.set(sample.pole_timestamp, sample.pole_angle[0])
        pole_angle_y.set(sample.pole_timestamp, sample.pole_angle[1])
        franka_effector_x.set(sample.franka_timestamp, sample.franka_effector_position[0])
        franka_effector_y.set(sample.franka_timestamp, sample.franka_effector_position[1])
        franka_effector_ddx.set(sample.franka_timestamp, (filtered_effector_dx - previous_filtered_effector_dx) / (sample.franka_timestamp - previous_franka_timestamp))
        franka_effector_ddy.set(sample.franka_timestamp, (filtered_effector_dy - previous_filtered_effector_dy) / (sample.franka_timestamp - previous_franka_timestamp))
        command_effector_ddx.set(sample.command_timestamp, sample.command_effector_acceleration[0])
        command_effector_ddy.set(sample.command_timestamp, sample.command_effector_acceleration[1])

        previous_filtered_effector_dx = filtered_effector_dx
        previous_filtered_effector_dy = filtered_effector_dy
        previous_franka_timestamp = sample.franka_timestamp

    # Feeding values
    if start_time is None:
        # Live
        rospy.init_node(namespace + "_plotter")
        sample_subscriber = rospy.Subscriber("/" + namespace + "/sample", Sample, callback)
        plotter.start()
        
    else:
        # Recorded
        duration = 3.0
        bag = rosbag.Bag(rospkg.RosPack().get_path("franka_pole") + "/temp/" + log_name + ".bag")
        for topic, sample, time in bag.read_messages(topics=["/" + namespace + "/sample"]):
            if sample.franka_timestamp < start_time: continue
            if sample.franka_timestamp > start_time + duration: break
            callback(sample)
        bag.close()
        plotter.start()