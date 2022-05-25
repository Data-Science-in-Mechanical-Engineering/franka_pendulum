#pragma once

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <geometry_msgs/PoseStamped.h>
#include <franka_pole/ControllerParameters.h>
#include <franka_pole/DebugSample.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <ros/node_handle.h>
#include <ros/time.h>

namespace franka_pole
{

class Controller;

//Configuration
enum class ControlType
{
  cartesian,
  torque
};

//Provides franka states
class FrankaState
{
private:
  Controller *_controller;
  double _time = 0.0;
  Eigen::Matrix<double, 3, 1> _effector_position = Eigen::Matrix<double, 3, 1>::Zero();
  Eigen::Matrix<double, 3, 1> _effector_velocity = Eigen::Matrix<double, 3, 1>::Zero();
  Eigen::Quaterniond _effector_orientation = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);

public:
  FrankaState(Controller *controller, ros::NodeHandle &node_handle);
  void update(double time); //Must be called in update()
  double get_time();
  Eigen::Matrix<double, 3, 1> get_effector_position();
  Eigen::Matrix<double, 3, 1> get_effector_velocity();
  Eigen::Quaterniond get_effector_orientation();
};

//Provides pole states
class PoleState
{
private:
  Controller *_controller;
  double _time = 0.0;
  double _angle = 0.0;
  double _dangle = 0.0;
  ros::Subscriber _pose_stamped_subscriber;
  void _update(const geometry_msgs::PoseStamped::ConstPtr &msg); //Must be called by ROS if not simulated

public:
  PoleState(Controller *controller, ros::NodeHandle &node_handle);
  void update(double time); //Must be called in update() if simulated
  double get_time();
  double get_angle();
  double get_dangle();
};

//Collects sample data
class Sampler
{
private:
  Controller *_controller;
  ros::Publisher _debug_sample_publisher;
  ros::Timer _debug_sample_timer;
  void _debug_sample_callback(const ros::TimerEvent &event);
public:
  Sampler(Controller *controller, ros::NodeHandle &node_handle);
};

class Controller : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface, franka_hw::FrankaStateInterface, hardware_interface::PositionJointInterface, hardware_interface::EffortJointInterface>
{
private:
  //Parameters
  std::string _arm_id = "panda";
  ControlType _control_type = ControlType::cartesian;
  bool _simulated = true;

  //Control
  double _a = 16.363880157470703 / 4;
  double _b = 9.875003814697266 / 4;
  double _c = 7.015979766845703 / 4;
  double _d = 11.86760425567627 / 4;
  double _desired_acceleration = 0.0;
  double _nullspace_stiffness = 0.0;                                                      //Used if _type == Cartesian
  double _nullspace_damping = 0.0;                                                        //Used if _type == Cartesian
  Eigen::Matrix<double, 6, 6> _cartesian_stiffness = Eigen::Matrix<double, 6, 6>::Zero(); //Used if _type == Cartesian
  Eigen::Matrix<double, 6, 6> _cartesian_damping = Eigen::Matrix<double, 6, 6>::Zero();   //Used if _type == Cartesian

  void _update(const franka_pole::ControllerParameters::ConstPtr &msg);

public:
  //ROS technical
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle; //franka model interface
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle; //franka state interface
  std::vector<hardware_interface::JointHandle> joint_handles; //genric joint interface

  //Components
  std::unique_ptr<FrankaState> franka_state;
  std::unique_ptr<PoleState> pole_state;
  std::unique_ptr<Sampler> sampler;

  //Functions
  bool is_simulated();
  ControlType control_type();
  double get_desired_acceleration();

  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
};

}  // namespace franka_pole