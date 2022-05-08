#!/usr/bin/python3
import rospy
import math
import numpy as np
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped
from franka_pole.msg import OptParams
from matplotlib import pyplot as plt
from matplotlib import animation

# Class containing and processing state of the robot arm
class FrankaState:
    def link_states_callback(self, link_states):
        effector_index = link_states.name.index('panda::panda_link7')
        self._effector_x = link_states.pose[effector_index].position.x
        self._effector_y = link_states.pose[effector_index].position.y
        self._effector_z = link_states.pose[effector_index].position.z
        self._effector_dx = link_states.twist[effector_index].linear.x
        self._effector_dy = link_states.twist[effector_index].linear.y
        self._effector_dz = link_states.twist[effector_index].linear.z

    def get_effector_y(self):
        return self._effector_y

    def get_effector_dy(self):
        return self._effector_dy

    def __init__(self, controller):
        self._controller = controller
        self._effector_x = None
        self._effector_y = None
        self._effector_z = None
        self._effector_dx = None
        self._effector_dy = None
        self._effector_dz = None
        link_states = rospy.wait_for_message("gazebo/link_states", LinkStates)
        self.link_states_callback(link_states)
        # Common callback with PoleState is launched by Controller

# Class containing and processing state of the pole
class PoleState:
    def link_states_callback(self, link_states):
        beam_index = link_states.name.index('panda::panda_pole_beam')
        angle = -Rotation.from_quat([
        link_states.pose[beam_index].orientation.x,
        link_states.pose[beam_index].orientation.y,
        link_states.pose[beam_index].orientation.z,
        link_states.pose[beam_index].orientation.w]).as_euler('zyx')[2]
        timestamp = rospy.get_time()
        self._dangle = (angle - self._angle) / (timestamp - self._timestamp)
        self._angle = angle
        self._timestamp = timestamp

    def get_angle(self):
        return self._angle

    def get_dangle(self):
        return self._dangle

    def __init__(self, controller):
        self._controller = controller
        self._angle = 0.0
        self._dangle = 0.0
        self._timestamp = 0.0
        link_states = rospy.wait_for_message("gazebo/link_states", LinkStates)
        self.link_states_callback(link_states)
        link_states = rospy.wait_for_message("gazebo/link_states", LinkStates)
        self.link_states_callback(link_states)
        # Common callback with FrankaState is launched by Controller

# Class responsible for sampling and plotting
class Plot:
    def animate_callback(self, i):
        # angle
        self._angle = np.roll(self._angle, -1)
        self._angle[-1] = self._controller.pole_state.get_angle()
        self._angle_line.set_data(self._time, self._angle)
        # dangle
        self._dangle = np.roll(self._dangle, -1)
        self._dangle[-1] = self._controller.pole_state.get_dangle()
        self._dangle_line.set_data(self._time, self._dangle)
        # effector_y
        self._effector_y = np.roll(self._effector_y, -1)
        self._effector_y[-1] = self._controller.franka_state.get_effector_y()
        self._effector_y_line.set_data(self._time, self._effector_y)
        # efector_dy
        self._effector_dy = np.roll(self._effector_dy, -1)
        self._effector_dy[-1] = self._controller.franka_state.get_effector_dy()
        self._effector_dy_line.set_data(self._time, self._effector_dy)
        # desired_acceleration
        self._desired_acceleration = np.roll(self._desired_acceleration, -1)
        self._desired_acceleration[-1] = self._controller.get_desired_acceleration()
        self._desired_acceleration_line.set_data(self._time, self._desired_acceleration)
        
        return self._angle_line, self._dangle_line, self._effector_y_line, self._effector_dy_line, self._desired_acceleration_line

    def __init__(self, controller):
        self._controller = controller

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
        self._angle = np.zeros(self._sample_time * self._sample_frequency)
        self._dangle = np.zeros(self._sample_time * self._sample_frequency)
        self._effector_y = np.zeros(self._sample_time * self._sample_frequency)
        self._effector_dy = np.zeros(self._sample_time * self._sample_frequency)
        self._desired_acceleration = np.zeros(self._sample_time * self._sample_frequency)

        # Starting animation
        self._animation = animation.FuncAnimation(self._figure, lambda i: self.animate_callback(i), interval=1000/self._sample_frequency, blit=True)

# Main class
class Controller:
    def optparams_callback(self, optparams):
        self._a = optparams.a
        self._b = optparams.b
        self._c = optparams.c
        self._d = optparams.d

    def link_states_callback(self, link_states):
        self.pole_state.link_states_callback(link_states)
        self.franka_state.link_states_callback(link_states)
        self._desired_acceleration = (
            self._a * self.pole_state.get_angle() +
            self._b * self.pole_state.get_dangle() +
            self._c * self.franka_state.get_effector_y() +
            self._d * self.franka_state.get_effector_dy()
        )

    def get_desired_acceleration(self):
        return self._desired_acceleration

    def spin(self):
        plt.show()

    def equilibrium_pose_publisher_callback(self):
        desired_pose = PoseStamped()
        desired_pose.header.frame_id = "panda_link0"
        desired_pose.header.stamp = rospy.Time(0)
        desired_pose.pose.orientation.x = 1
        desired_pose.pose.orientation.y = 0
        desired_pose.pose.orientation.z = 0
        desired_pose.pose.orientation.w = 0
        desired_pose.pose.position.x = 0.5
        desired_pose.pose.position.y = self.franka_state.get_effector_y() + self._desired_acceleration
        desired_pose.pose.position.z = 0.5
        desired_pose.pose.position.x = max(self._range_x[0], min(self._range_x[1], desired_pose.pose.position.x))
        desired_pose.pose.position.y = max(self._range_y[0], min(self._range_y[1], desired_pose.pose.position.y))
        desired_pose.pose.position.z = max(self._range_z[0], min(self._range_z[1], desired_pose.pose.position.z))
        self._equilibrium_pose_publisher.publish(desired_pose)

    def __init__(self):
        # Initializing node
        rospy.init_node('controller')

        # Setting parameters
        self._range_x = [-0.6, 0.6]
        self._range_y = [-0.6, 0.6]
        self._range_z = [0.05, 0.9]
        self._a = 16.363880157470703 / 4
        self._b = 9.875003814697266 / 4
        self._c = 7.015979766845703 / 4
        self._d = 11.86760425567627 / 4
        self._desired_acceleration = 0

        # Creating components
        self.franka_state = FrankaState(self)
        self.pole_state = PoleState(self)
        self.plot = Plot(self)

        # Starting common callback for franka_state and pole_state
        self._optparams_subscriber = rospy.Subscriber('/optparams', OptParams, lambda optparams: self.optparams_callback(optparams))

        # Starting common callback for franka_state and pole_state
        self._link_states_subscriber = rospy.Subscriber('/gazebo/link_states', LinkStates, lambda link_states: self.link_states_callback(link_states))

        # Starting publisher
        self._equilibrium_pose_publisher = rospy.Publisher("/cartesian_impedance_example_controller/equilibrium_pose", PoseStamped, queue_size=10)
        self._equilibrium_pose_publisher_timer = rospy.Timer(rospy.Duration(0.005), lambda msg: self.equilibrium_pose_publisher_callback())
        
        

if __name__ == '__main__':
    # Creating controller
    controller = Controller()

    # Starting controller
    controller.spin()