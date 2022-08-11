# Welcome to `franka_pole`!
Here you will find a ROS package that is used for control of [Franka Emika Panda](https://www.franka.de/) robot. The library's main goal is to develop an algorithm that will allow the robot to hold an inverse pendulum. The secondary goal is to provide simple and user-friendly development and demonstration environment.

### Contents
1. [Welcome to franka_pole](#welcome-to-franka_pole)
2. [Contents](#contents)
3. [Requirements](#requirements)
4. [Building](#building)
5. [Usage](#usage)
6. [Parameters](#parameters)
7. [Scripts](#scripts)
8. [Example](#example)
9. [Documentation](#documentation)
10. [Contributors](#contributors)

### Requirements
`franka_pole` requires:
 - [ROS](http://wiki.ros.org/) core packages
 - [franka_ros](https://frankaemika.github.io/)
 - [pinocchio](https://stack-of-tasks.github.io/pinocchio)
 - [Doxygen](https://www.doxygen.nl/)

### Building
An installation script for Debian-based systems follows:
```
ROS_DISTRO=<YOUR ROS DISTRIBUTION>
sudo apt install ros-${ROS_DISTRO}-franka-ros                   # Installing franka_ros package
sudo apt install ros-${ROS_DISTRO}-pinocchio                    # Installing pinocchio package
sudo apt install ros-${ROS_DISTRO}-rosdoc-lite                  # Installing rosdoc-lite
sudo apt install ros-${ROS_DISTRO}-fkie-master-sync             # Installing fkie-master-sync
sudo apt install doxygen                                        # Installing Doxygen
if [ ! -d ~/catkin_ws/src ]; then mkdir -p ~/catkin_ws/src; fi  # Creating ROS workspace
cd ~/catkin_ws/src                                              # Setting directory to sources
git clone https://github.com/kyrylo-sovailo/franka_pole         # Downloading source code
cd ..                                                           # Setting directory to workspace
source /opt/ros/${ROS_DISTRO}/setup.bash                        # Including ROS
catkin_make                                                     # Building ROS workspace
source ~/catkin_ws/devel/setup.bash                             # Including the workspace
```

You may also want to add the following lines to your `~/.bashrc` in order to execute them automatically when you enter the terminal:
```
source /opt/ros/<YOUR ROS DISTRIBUTION>/setup.bash
source ~/catkin_ws/devel/setup.bash
```

### Usage
All long-term use use cases are covered in one `.launch` file. In can be started with a command:
```
roslaunch franka_pole franka_pole.launch <ARGUMENTS>
```
Arguments need to be given in form `name:=value`. The list of possible arguments:
 - `source` - Source of the data: `gazebo` for simulation (default), `robot` for real hardware experiment, `record` for playing previously recorded record
 - `model` - Model of the robot: `0D` for bare robot (default), `1D` for one-DOF pole, `2D` and `2Db` for two models of two-DOF poles
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

### Parameters
`franka_pole` has extensive parametrization system. When a program is launched, parameters are read in following order:
1. `config/parameters.yaml` is read as a **template**. The file contains most of the settings
2. `config/override/*.yaml` are conditionally read to **override** some parameters from the template. For example, `config/override/acceleration+0D.yaml` is read when `acceleration:=true` and `model:=0D`. More specific files override more general ones
3. Then parameters are read by the `franka_pole` itself, with all `.NaN` values **fall back** to some previously configured values
4. After parameters are read, they are published to `/<NAMESPACE>/command_parameters` topic. This topic can be read and published to, implementing **dynamic reconfigure** feature

It may be also confusing that `franka_pole` has several stiffness/damping parameters:
 - `outbound_*_stiffness|damping` - applied when the effector is outside of the boundaries
 - `translation_stiffness|damping` - conventional cartesian control that tries to move the end effector to target position with target velocity. In case of acceleration controller, these position and velocity are obtained by integration of the acceleration
 - `rotation_stiffness|damping` - conventional cartesian rotational control, tries to reach effector's target orientation with zero angular velocity
 - `joint_stiffness|damping` - joint-space control, where target joint positions and velocities are obtained by inverse kinematic algorithm. Inverse kinematics is run every time, for every current cartesian target
 - `nullspace_stiffness|damping` - joint-space control that tries to reach target joint position with zero velocity. Inverse kinematics is run only once, for target cartesian position
 - `dynamics` - inverse dynamics, tries to control robot's acceleration directly with torques. The output of inverse dynamics algorithm is multiplied by `dynamics` factor
 - `hardware_reset_stiffness|damping` - joint-space control to be used when returning the robot to it's original position with hardware reset

### Scripts
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

### Example
The package includes an example ROS package that depends on `franka_pole`. Copy `example` directory to `~/catkin_ws/src/franka_pole_example` and your example package is ready. The example demonstates basic usage of `franka_pole` package from `.launch`, `CMakeLists.txt` and C++ files. Be free to use `franka_pole_example` as a template for your project and replace `franka_pole_example` with your desired name.

### Documentation
The documentation can be generated with a command:
```
rosdoc_lite $(rospack find franka_pole) # Generate documentation
firefox $(rospack find franka_pole)/doc/html/index.html # Open the documentation in Firefox
```
The documentation covers all C++ classes and interfaces important for development.

### Contributors
 - Kyrylo Sovailo