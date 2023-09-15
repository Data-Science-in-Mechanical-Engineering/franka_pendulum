#pragma once

#include <franka_pendulum/CommandReset.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#ifdef FRANKA_POLE_VELOCITY_INTERFACE
    #include <franka_hw/franka_cartesian_command_interface.h>
    #include <franka_hw/franka_state_interface.h>
#else
    #include <hardware_interface/joint_command_interface.h>
#endif
#include <pluginlib/class_list_macros.h>

#include <ros/time.h>
#include <Eigen/Dense>
#include <semaphore.h>
#include <mutex>

namespace franka_pendulum
{
    class Parameters;
    class FrankaModel;
    class FrankaState;
    class PendulumState;
    class Publisher;

    ///Low-level controller, responsible for initialization of components, timing, reset and other low-level entities
    class Controller : public controller_interface::MultiInterfaceController<
        #ifdef FRANKA_POLE_VELOCITY_INTERFACE
            franka_hw::FrankaVelocityCartesianInterface, franka_hw::FrankaStateInterface
        #else
            hardware_interface::EffortJointInterface, hardware_interface::PositionJointInterface
        #endif
        >
    {
    private:
        //Technical
        hardware_interface::RobotHW *_robot_hw;
        ros::NodeHandle _node_handle;

        //Time
        ros::Time _time;
        unsigned int _franka_period_counter;
        unsigned int _pendulum_period_counter;
        unsigned int _command_period_counter;
        unsigned int _publish_period_counter;
        
        //Reset
        enum class ResetMode
        {
            normal,
            hardware_reset,
            software_reset
        };
        ResetMode _reset_mode;
        ros::Subscriber _reset_subscriber;

        //Software reset
        sem_t *_software_reset_semaphore;

        //Hardware reset
        ros::Publisher _hardware_reset_publisher;
        Eigen::Matrix<double, 3, 1> _hardware_reset_start_position;
        Eigen::Quaterniond _hardware_reset_start_orientation;
        Eigen::Matrix<double, 7, 1> _hardware_reset_start_positions;
        Eigen::Matrix<double, 7, 1> _hardware_reset_end_positions; //Inverse kinematics cache
        double _hardware_reset_time;

        //Output
        #ifdef FRANKA_POLE_VELOCITY_INTERFACE
            Eigen::Matrix<double, 6, 1> _velocity;
        #else
            Eigen::Matrix<double, 7, 1> _torque;
        #endif        

        void _initiate_hardware_reset();
        void _initiate_software_reset();
        void _initiate_normal_mode();
        void _callback(const franka_pendulum::CommandReset::ConstPtr &msg);

    protected:
        //System handling
        ///Initializes low-level controller
        ///@param robot_hw `hardware_interface::RobotHW` object
        ///@param node_handle ROS node handle
        ///@return `true` if initialization was successfull
        ///@warning Internal use only
        bool _init_level0(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle);
        ///Notifies low-level controller about first update
        ///@param time Current time
        ///@warning Internal use only
        void _starting_level0(const ros::Time &time);
        ///Notifies low-level controller about update
        ///@param time Current time
        ///@param period Time from previous update
        ///@warning Internal use only
        void _update_level0(const ros::Time &time, const ros::Duration &period);

        //Interface for higher level controllers
        #ifdef FRANKA_POLE_VELOCITY_INTERFACE
            ///Initializes higher level controllers
            ///@param robot_hw `hardware_interface::RobotHW` object
            ///@param node_handle ROS node handle
            ///@return `true` if initialization was successfull
            virtual bool _init_level1(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) = 0;
            ///Gets torque command from higher level controllers
            ///@param time Current time
            ///@param period Time from previous update
            ///@return cartesian velocity
            virtual Eigen::Matrix<double, 6, 1> _get_velocity_level1(const ros::Time &time, const ros::Duration &period) = 0;
        #else
            ///Initializes higher level controllers
            ///@param robot_hw `hardware_interface::RobotHW` object
            ///@param node_handle ROS node handle
            ///@return `true` if initialization was successfull
            virtual bool _init_level1(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) = 0;
            ///Gets torque command from higher level controllers
            ///@param time Current time
            ///@param period Time from previous update
            ///@return joint torques
            virtual Eigen::Matrix<double, 7, 1> _get_torque_level1(const ros::Time &time, const ros::Duration &period) = 0;
        #endif

    public:
        //Components
        std::mutex mutex;                   ///< System-wide mutex
        Parameters *parameters = nullptr;   ///< Parameter set
        FrankaModel *franka_model = nullptr;///< Model of the robot
        FrankaState *franka_state = nullptr;///< State of the robot
        PendulumState *pendulum_state = nullptr;    ///< State of the pendulum
        Publisher *publisher = nullptr;     ///< Publisher
        
        virtual ~Controller();  ///< Destroys the controller
    };
}

#define FRANKA_POLE_CONTROLLER_DECLARATION() \
public: \
bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override; \
void starting(const ros::Time &time) override; \
void update(const ros::Time &time, const ros::Duration &period) override;

#define FRANKA_POLE_CONTROLLER_IMPLEMENTATION(__NAME__) \
PLUGINLIB_EXPORT_CLASS(__NAME__, controller_interface::ControllerBase) \
bool __NAME__::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) { return franka_pendulum::Controller::_init_level0(robot_hw, node_handle); } \
void __NAME__::starting(const ros::Time &time) { franka_pendulum::Controller::_starting_level0(time); } \
void __NAME__::update(const ros::Time &time, const ros::Duration &period) { franka_pendulum::Controller::_update_level0(time, period); }

/** @mainpage Welcome to `franka_pendulum`!
Here you will find a ROS package that is used for control of [Franka Emika Panda](https://www.franka.de/) robot. The library's main goal is to develop an algorithm that will allow the robot to hold an inverse pendulum. The secondary goal is to provide simple and user-friendly development and demonstration environment.

@tableofcontents

@section Requirements
`franka_pendulum` requires:
 - [ROS](http://wiki.ros.org/) core packages
 - [franka_ros](https://frankaemika.github.io/)
 - [pinocchio](https://stack-of-tasks.github.io/pinocchio)
 - [Doxygen](https://www.doxygen.nl/)

@section Building
An installation script for Debian-based systems follows:
@code{.sh}
ROS_DISTRO=<YOUR ROS DISTRIBUTION>
sudo apt install ros-${ROS_DISTRO}-franka-ros                   # Installing franka_ros package
sudo apt install ros-${ROS_DISTRO}-pinocchio                    # Installing pinocchio package
sudo apt install ros-${ROS_DISTRO}-rosdoc-lite                  # Installing rosdoc-lite
sudo apt install ros-${ROS_DISTRO}-fkie-master-sync             # Installing fkie-master-sync
sudo apt install doxygen                                        # Installing Doxygen
if [ ! -d ~/catkin_ws/src ]; then mkdir -p ~/catkin_ws/src; fi  # Creating ROS workspace
cd ~/catkin_ws/src                                              # Setting directory to sources
git clone https://github.com/Data-Science-in-Mechanical-Engineering/franka_pendulum         # Downloading source code
cd ..                                                           # Setting directory to workspace
source /opt/ros/${ROS_DISTRO}/setup.bash                        # Including ROS
catkin_make                                                     # Building ROS workspace
source ~/catkin_ws/devel/setup.bash                             # Including the workspace
@endcode

You may also want to add the following lines to your `~/.bashrc` in order to execute them automatically when you enter the terminal:
@code{.sh}
source /opt/ros/<YOUR ROS DISTRIBUTION>/setup.bash
source ~/catkin_ws/devel/setup.bash
@endcode

@section Usage
All long-term use use cases are covered in one `.launch` file. In can be started with a command:
@code{.sh}
roslaunch franka_pendulum franka_pendulum.launch <ARGUMENTS>
@endcode{.sh}
Arguments need to be given in form `name:=value`. The list of possible arguments:
 - `source` - Source of the data: `gazebo` for simulation (default), `robot` for real hardware experiment, `record` for playing previously recorded record
 - `model` - Model of the robot: `0D` for bare robot (default), `1D` for one-DOF pendulum, `2D` and `2Db` for two models of two-DOF pendulums
 - `type` - Type of the controller: `test` for test controllers (default), `simple` for integrated C++ PD controller, `python` for included Python PD controller, `external` for receiving commands via ROS topics
 - `acceleration` - `true` if acceleration controller should be used (default), `false` to use position tracking controller
 - `paused` - `true` if simulation should be stopped instantly after beginning, `false` if should not (default) [Takes affect only if `source:=gazebo`]

 - `gui` - `true` if Gazebo GUI should be launched (default) (Takes affect only if `source:=gazebo`)
 - `plotter` - `true` if plotting script should be launched (default)
 - `rviz` - `true` if RVIZ should be launched, `false` if should not (default)
 - `resetter` - `true` if resetter script should be launched, `false` if should not (default)
 - `logger` - `true` if logging script be should launched, `false` if should not (default). Log format is human-readable and easily configurable
 - `recorder` - `true` if recorder script be launched, `false` if should not (default). Record format not human-readable and not configurable, but record files use less system resources and can be replayed

 - `namespace` - Prefix for all node names, parameter names and ROS topic names
 - `log_name` - Name of the file produced by the logger
 - `record_name` - Name of the file produced by the recorder
 - `arm_id` - Name of the arm
 - `arm_ip` - IP address of the real Franka robot
 - `vicon_ip` - IP address of the real VICON system

@section Parameters
`franka_pendulum` has extensive parametrization system. When a program is launched, parameters are read in following order:
1. `config/parameters.yaml` is read as a **template**. The file contains most of the settings
2. `config/override/*.yaml` are conditionally read to **override** some parameters from the template. For example, `config/override/acceleration+0D.yaml` is read when `acceleration:=true` and `model:=0D`. More specific files override more general ones
3. Then parameters are read by the `franka_pendulum` itself, with all `.NaN` values **fall back** to some previously configured values
4. After parameters are read, they are published to `/<NAMESPACE>/command_parameters` topic. This topic can be read and published to, implementing **dynamic reconfigure** feature

It may be also confusing that `franka_pendulum` has several stiffness/damping parameters:
 - `outbound_*_stiffness|damping` - applied when the effector is outside of the boundaries
 - `translation_stiffness|damping` - conventional cartesian control that tries to move the end effector to target position with target velocity. In case of acceleration controller, these position and velocity are obtained by integration of the acceleration
 - `rotation_stiffness|damping` - conventional cartesian rotational control, tries to reach effector's target orientation with zero angular velocity
 - `joint_stiffness|damping` - joint-space control, where target joint positions and velocities are obtained by inverse kinematic algorithm. Inverse kinematics is run every time, for every current cartesian target
 - `nullspace_stiffness|damping` - joint-space control that tries to reach target joint position with zero velocity. Inverse kinematics is run only once, for target cartesian position
 - `dynamics` - inverse dynamics, tries to control robot's acceleration directly with torques. The output of inverse dynamics algorithm is multiplied by `dynamics` factor
 - `hardware_reset_stiffness|damping` - joint-space control to be used when returning the robot to it's original position with hardware reset

@section Scripts
Python scripts are more of an example scripts, they are not meant to include them and develop upon them. Still, here is some helpful information:
 - `acceleration_controller.py` and `position_controller.py` - Python alternatives of `SimpleAccelerationController` and `SimplePositionController` respectively. These scripts demonstrate control over ROS topics
Several scripts that can be launched manually:
 - `logger.py` - logs data to a file in human-readable format. Demonstrates subscribing to ROS topics and reading from them
 - `plotter.py` - plots data in real time. Demonstrates subscribing to ROS topics and data plotting. Provides importable interace. Can be called by user to plot data from recorded file instead of plotting real time
 - `parts.py` - calculates masses and moments of inertia of parts and prints as `URDF` model. Provides importable interace
 - `lqr.py` - using masses and moments of inertia analytically calculates appropriate control gains with LQR algorithm
 - `reconfigure.py` - demonstrates dynamic reconfigure feature
 - `optimize.py` - demonstrates dynamic reconfigure feature and advanced multi-threading code
 - `resetter.py` - small utility to periodically reset the simulation
 - `run.sh` - Starts and stops the simulation. Demonstrates background usage of `roslaunch`
 - `run_parallel.sh` - Starts and stops the simulation and the real robot in paralell. Demonstrates multi-master usage
 - `rviz.sh` - Starts RVIZ with arbitrary arm name. For internal usage

@section Example
The package includes an example ROS package that depends on `franka_pendulum`. Copy `example` directory to `~/catkin_ws/src/franka_pendulum_example` and your example package is ready. The example demonstates basic usage of `franka_pendulum` package from `.launch`, `CMakeLists.txt` and C++ files. Be free to use `franka_pendulum_example` as a template for your project and replace `franka_pendulum_example` with your desired name.

@section Documentation
The documentation can be generated with a command:
@code{.sh}
rosdoc_lite $(rospack find franka_pendulum) # Generate documentation
firefox $(rospack find franka_pendulum)/doc/html/index.html # Open the documentation in Firefox
@endcode{.sh}
The documentation covers all C++ classes and interfaces important for development.

@section Contributors
 - Kyrylo Sovailo
 - Shiming He
 - Alexander von Rohr
*/