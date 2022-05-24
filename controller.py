#!/usr/bin/python3
import rospy
from scipy.spatial.transform import Rotation
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped
from franka_pole.msg import ControllerParameters, DebugSample

# Class containing and processing state of the robot arm
class FrankaState:
    #Sets data from link states, called from Controller (gazebo only)
    def link_states_callback(self, link_states):
        effector_index = link_states.name.index(self._controller.get_arm_id() + "::" + self._controller.get_arm_id() + "_link7")
        self._effector_x = link_states.pose[effector_index].position.x
        self._effector_y = link_states.pose[effector_index].position.y
        self._effector_z = link_states.pose[effector_index].position.z
        self._effector_dx = link_states.twist[effector_index].linear.x
        self._effector_dy = link_states.twist[effector_index].linear.y
        self._effector_dz = link_states.twist[effector_index].linear.z

    #Sets data from joint states, called from Controller (real robot only)
    def joint_states_callback(self, joint_states):
        pass

    #Gets effector y position
    def get_effector_y(self):
        return float(self._effector_y)

    #Gets effector y velocity
    def get_effector_dy(self):
        return float(self._effector_dy)

    #Creates FrankaState
    def __init__(self, controller):
        self._controller = controller
        self._effector_x = None
        self._effector_y = None
        self._effector_z = None
        self._effector_dx = None
        self._effector_dy = None
        self._effector_dz = None
        if controller.get_gazebo():
            link_states = rospy.wait_for_message("gazebo/link_states", LinkStates)
            self.link_states_callback(link_states)
        else:
            pass
        # Common callback with PoleState is launched by Controller

# Class containing and processing state of the pole
class PoleState:
    #Sets data from link states, called from Controller (gazebo only)
    def link_states_callback(self, link_states):
        beam_index = link_states.name.index(self._controller.get_arm_id() + "::" + self._controller.get_arm_id() + "_upper")
        angle = -Rotation.from_quat([
        link_states.pose[beam_index].orientation.x,
        link_states.pose[beam_index].orientation.y,
        link_states.pose[beam_index].orientation.z,
        link_states.pose[beam_index].orientation.w]).as_euler('zyx')[2]
        timestamp = rospy.get_time()
        self._dangle = (angle - self._angle) / (timestamp - self._timestamp)
        self._angle = angle
        self._timestamp = timestamp

    #Sets data from transform called from Controller (real robot only)
    def transform_stamped_callback(self, transform_stamped):
        angle = -Rotation.from_quat([
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z,
        transform_stamped.transform.rotation.w]).as_euler('zyx')[2]
        timestamp = rospy.get_time()
        self._dangle = (angle - self._angle) / (timestamp - self._timestamp)
        self._angle = angle
        self._timestamp = timestamp

    #Gets pole angle
    def get_angle(self):
        return float(self._angle)

    #Gets pole angular velocity
    def get_dangle(self):
        return float(self._dangle)

    #Creates PoleState
    def __init__(self, controller):
        self._controller = controller
        self._angle = 0.0
        self._dangle = 0.0
        self._timestamp = 0.0
        if controller.get_gazebo():
            link_states = rospy.wait_for_message("gazebo/link_states", LinkStates)
            self.link_states_callback(link_states)
            link_states = rospy.wait_for_message("gazebo/link_states", LinkStates)
            self.link_states_callback(link_states)
        else:
            pass
        # Common callback with FrankaState is launched by Controller

# Class responsible for sampling and publishing data
class Sampler:
    #Sets optimization parameters, called by user
    def sample_timer_callback(self):
        sample = DebugSample()
        sample.angle = self._controller.pole_state.get_angle()
        sample.dangle = self._controller.pole_state.get_dangle()
        sample.effector_y = self._controller.franka_state.get_effector_y()
        sample.effector_dy = self._controller.franka_state.get_effector_dy()
        sample.desired_acceleration = self._controller.get_desired_acceleration()
        self._sample_puslisher.publish(sample)

    #Creates Sampler
    def __init__(self, controller):
        self._controller = controller

        # Configuration
        self._sample_frequency = 20

        # Creating timer
        self._sample_puslisher = rospy.Publisher("/franka_pole/debug_samples", DebugSample, queue_size=10)
        self._sample_publisher_timer = rospy.Timer(rospy.Duration(1 / self._sample_frequency), lambda msg: self.sample_timer_callback())

# Main class
class Controller:
    #Sets optimization parameters, called by user
    def parameters_callback(self, optparams):
        self._a = optparams.a
        self._b = optparams.b
        self._c = optparams.c
        self._d = optparams.d

    #Sets franka state and pole state from link states, called from Controller (gazebo only)
    def link_states_callback(self, link_states):
        self.pole_state.link_states_callback(link_states)
        self.franka_state.link_states_callback(link_states)
        self.calculate()

    #Sets franka state from joint states, called from Controller (real robot only)
    def joint_states_callback(self, joint_states):
        pass

    #Sets pole state from transform, called from Controller (real robot only)
    def transform_stamped_callback(self, transform_stamped):
        pass

    #Calculates desired acceleration, called from Controller callbacks
    def calculate(self):
        self._desired_acceleration = (
            self._a * self.pole_state.get_angle() +
            self._b * self.pole_state.get_dangle() +
            self._c * self.franka_state.get_effector_y() +
            self._d * self.franka_state.get_effector_dy()
        )

    #Publishes desired acceleration, called by Timer
    def control_callback(self):
        if self._gazebo:
            if self._type == "position":
                desired_pose = PoseStamped()
                desired_pose.header.frame_id = self._arm_id + "_link0"
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
                self._cartesian_target_publisher.publish(desired_pose)
            elif self._type == "velocity":
                pass
            elif self._type == "torque":
                pass
        else:
            if self._type == "position":
                pass
            elif self._type == "velocity":
                pass
            elif self._type == "torque":
                pass

    #Gets desired acceleration
    def get_desired_acceleration(self):
        return float(self._desired_acceleration)

    #Gets arm id
    def get_arm_id(self):
        return self._arm_id

    #Gets control type
    def get_type(self):
        return self._type

    #Gets if simulated
    def get_gazebo(self):
        return self._gazebo

    #Starts controller
    def spin(self):
        rospy.spin()

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
        self._arm_id = rospy.get_param("~arm_id")
        self._type = rospy.get_param("~controller")
        self._gazebo = bool(rospy.get_param("~gazebo"))
        self._desired_acceleration = 0

        # Creating components
        self.franka_state = FrankaState(self)
        self.pole_state = PoleState(self)
        self.sampler = Sampler(self)

        # Starting common callback for franka_state and pole_state
        self._optparams_subscriber = rospy.Subscriber('/franka_pole/controller_parameters', ControllerParameters, lambda params: self.parameters_callback(params))

        # Starting common callback for franka_state and pole_state
        if self._gazebo:
            self._link_states_subscriber = rospy.Subscriber('/gazebo/link_states', LinkStates, lambda link_states: self.link_states_callback(link_states))
        else:
            pass

        # Starting controller
        if self._type == "position":
            self._cartesian_target_publisher = rospy.Publisher("/franka_pole/cartesian_target", PoseStamped, queue_size=10)
        elif self._type == "velocity":
            pass
        elif self._type == "torque":
            pass

        self._control_timer = rospy.Timer(rospy.Duration(0.005), lambda msg: self.control_callback())

if __name__ == '__main__':
    # Creating controller
    controller = Controller()

    # Starting controller
    controller.spin()