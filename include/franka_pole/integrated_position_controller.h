#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <franka_pole/ControllerParameters.h>
#include <franka_pole/DebugSample.h>

namespace franka_pole
{

class IntegratedPositionController;

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
  IntegratedPositionController *_controller;
  double _time = 0.0;

  Eigen::Vector3d _effector_position;
  Eigen::Matrix<double, 6, 1> _effector_velocity;
  Eigen::Quaterniond _effector_orientation;
  Eigen::Affine3d _effector_transform;
  Eigen::Matrix<double, 6, 7> _effector_jacobian;

  Eigen::Matrix<double, 7, 1> _joint_positions;
  Eigen::Matrix<double, 7, 1> _joint_velocities;
  Eigen::Matrix<double, 7, 1> _coriolis;

public:
  FrankaState(IntegratedPositionController *controller, ros::NodeHandle &node_handle);
  void update(double time); //Must be called in update()
  double get_time();
  
  Eigen::Vector3d get_effector_position();
  Eigen::Matrix<double, 6, 1> get_effector_velocity();
  Eigen::Quaterniond get_effector_orientation();
  Eigen::Affine3d get_effector_transform();
  Eigen::Matrix<double, 6, 7> get_effector_jacobian();

  Eigen::Matrix<double, 7, 1> get_joint_positions();
  Eigen::Matrix<double, 7, 1> get_joint_velocities();
  Eigen::Matrix<double, 7, 1> get_coriolis();
};

//Provides pole states
class PoleState
{
private:
  IntegratedPositionController *_controller;
  double _time = 0.0;
  double _angle = 0.0;
  double _dangle = 0.0;
  ros::Subscriber _pose_stamped_subscriber;
  void _update(const geometry_msgs::PoseStamped::ConstPtr &msg); //Must be called by ROS if not simulated

public:
  PoleState(IntegratedPositionController *controller, ros::NodeHandle &node_handle);
  void update(double time); //Must be called in update() if simulated
  double get_time();
  double get_angle();
  double get_dangle();
};

//Collects sample data
class Sampler
{
private:
  IntegratedPositionController *_controller;
  ros::Publisher _debug_sample_publisher;
  ros::Timer _debug_sample_timer;
  void _debug_sample_callback(const ros::TimerEvent &event);
public:
  Sampler(IntegratedPositionController *controller, ros::NodeHandle &node_handle);
};

class IntegratedPositionController : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface, hardware_interface::EffortJointInterface, hardware_interface::PositionJointInterface, franka_hw::FrankaStateInterface>
{
private:
  //Parameters
  std::string _arm_id = "panda";
  ControlType _control_type = ControlType::cartesian;
  bool _simulated = true;

  //Control
  double _a = 16.363880157470703 / 30;
  double _b = 9.875003814697266 / 30;
  double _c = 7.015979766845703 / 30;
  double _d = 11.86760425567627 / 30;
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
