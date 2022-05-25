#include "franka_pole_controller.h"

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

//================================================================== FrankaState ==================================================================

franka_pole::FrankaState::FrankaState(Controller *controller, ros::NodeHandle &)
{
  _controller = controller;
}

void franka_pole::FrankaState::update(double time)
{
  _time = time;

  franka::RobotState robot_state = _controller->state_handle->getRobotState();
  Eigen::Matrix<double, 6, 7> jacobian(Eigen::Matrix<double, 6, 7>(_controller->model_handle->getZeroJacobian(franka::Frame::kEndEffector).data()));
  Eigen::Matrix<double, 7, 1> dq(Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data()));
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  _effector_position = transform.translation();
  _effector_velocity = (jacobian * dq).segment<3>(0);
  _effector_orientation = Eigen::Quaterniond(transform.linear());
}

double franka_pole::FrankaState::get_time()
{
  return _time;
}

Eigen::Matrix<double, 3, 1> franka_pole::FrankaState::get_effector_position()
{
  return _effector_position;
}

Eigen::Matrix<double, 3, 1> franka_pole::FrankaState::get_effector_velocity()
{
  return _effector_velocity;
}

Eigen::Quaterniond franka_pole::FrankaState::get_effector_orientation()
{
  return _effector_orientation;
}

//================================================================== PoleState ==================================================================

void franka_pole::PoleState::_update(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  //todo
}

franka_pole::PoleState::PoleState(Controller *controller, ros::NodeHandle &node_handle)
{
  _controller = controller;
  if (_controller->is_simulated()) _pose_stamped_subscriber = node_handle.subscribe("/vicon/Pole_1/Pole_1", 20, &PoleState::_update, this, ros::TransportHints().reliable().tcpNoDelay());
}

void franka_pole::PoleState::update(double time)
{
  _time = time;
  _angle = _controller->joint_handles.back().getPosition();
  _dangle = _controller->joint_handles.back().getVelocity();
}

double franka_pole::PoleState::get_time()
{
  return _time;
}

double franka_pole::PoleState::get_angle()
{
  return _angle;
}

double franka_pole::PoleState::get_dangle()
{
  return _dangle;
}

//================================================================== Sampler ==================================================================

void franka_pole::Sampler::_debug_sample_callback(const ros::TimerEvent &event)
{
  DebugSample sample;
  sample.message_time = event.current_real.toSec();

  sample.pole_time = _controller->pole_state->get_time();
  sample.pole_angle = _controller->pole_state->get_angle();
  sample.pole_dangle = _controller->pole_state->get_dangle();

  sample.franka_time = _controller->franka_state->get_time();
  sample.franka_effector_y = _controller->franka_state->get_effector_position()(1);
  sample.franka_effector_dy = _controller->franka_state->get_effector_velocity()(1);

  sample.desired_acceleration = _controller->get_desired_acceleration();

  _debug_sample_publisher.publish(sample);
}

franka_pole::Sampler::Sampler(Controller *controller, ros::NodeHandle &node_handle)
{
  _controller = controller;
  _debug_sample_publisher = node_handle.advertise<franka_pole::DebugSample>("/franka_pole/samples", 20);
  _debug_sample_timer = node_handle.createTimer(ros::Duration(0.01), &franka_pole::Sampler::_debug_sample_callback, this);
}

//================================================================== Controller ==================================================================

bool franka_pole::Controller::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
  //Parameters
  /*
  if (!node_handle.getParam("arm_id", _arm_id))
  {
    ROS_ERROR_STREAM("Controller: Could not read parameter arm_id");
    return false;
  }

  std::string control_type_str;
  if (!node_handle.getParam("control_type", control_type_str)) { ROS_ERROR_STREAM("Controller: Could not read parameter control_type"); return false; }
  if (control_type_str == "cartesian") _control_type = ControlType::cartesian;
  else if (control_type_str == "torque") _control_type = ControlType::torque;
  else { ROS_ERROR_STREAM("Controller: Invalid controller_type parameter"); return false; }

  std::string simulated_str;
  if (!node_handle.getParam("simulated", simulated_str)) { ROS_ERROR_STREAM("Controller: Could not read parameter simulated"); return false; }
  if (simulated_str == "true") _simulated = true;
  else if (simulated_str == "fasle") _simulated = false;
  else { ROS_ERROR_STREAM("Controller: Invalid controller_type simulated"); return false; }
  */
  //Model interface
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) { ROS_ERROR_STREAM("Controller: Error getting model interface from hardware"); return false; }
  try { model_handle = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(_arm_id + "_model")); }
  catch (hardware_interface::HardwareInterfaceException& ex) { ROS_ERROR_STREAM("Controller: Exception getting model handle from interface: " << ex.what()); return false; }

  //State interface
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) { ROS_ERROR_STREAM("Controller: Error getting state interface from hardware"); return false; }
  try { state_handle = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(_arm_id + "_robot")); }
  catch (hardware_interface::HardwareInterfaceException &ex) { ROS_ERROR_STREAM("Controller: Exception getting state handle from interface: " << ex.what()); return false; }

  //Joint effort interface
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) { ROS_ERROR_STREAM("Controller: Error getting effort joint effort interface from hardware"); return false; }
  for (size_t i = 0; i < 7; ++i)
  {
    try { joint_handles.push_back(effort_joint_interface->getHandle(_arm_id + "_joint" + std::to_string(i + 1))); }
    catch (const hardware_interface::HardwareInterfaceException &ex) { ROS_ERROR_STREAM("Controller: Exception getting joint handles: " << ex.what()); return false; }
  }

  //Joint interface
  auto* position_joint_interface = robot_hw->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface == nullptr) { ROS_ERROR_STREAM("Controller: Error getting effort joint interface from hardware"); return false; }
  try { joint_handles.push_back(position_joint_interface->getHandle(_arm_id + "_lower_upper")); }
  catch (const hardware_interface::HardwareInterfaceException &ex) { ROS_ERROR_STREAM("Controller: Exception getting joint handles: " << ex.what()); return false; }

  ROS_WARN_STREAM("HERE1");
  //Control
  _cartesian_stiffness.setZero();
  _cartesian_stiffness.diagonal().segment<3>(0) = Eigen::Matrix<double, 3, 1>::Ones() * 200.0;
  _cartesian_stiffness.diagonal().segment<3>(3) = Eigen::Matrix<double, 3, 1>::Ones() * 10.0;
  _nullspace_stiffness = 0.5;
  _nullspace_damping = 2.0 * sqrt(_nullspace_stiffness);
  _cartesian_damping = 2.0 * _cartesian_stiffness.array().sqrt().matrix();
  return true;

  //Components
  franka_state = std::make_unique<FrankaState>(this, node_handle);
  pole_state  = std::make_unique<PoleState>(this, node_handle);
  sampler  = std::make_unique<Sampler>(this, node_handle);
}

void franka_pole::Controller::starting(const ros::Time &)
{
}

void franka_pole::Controller::update(const ros::Time &, const ros::Duration &)
{
  franka_state->update(0.0);
  pole_state->update(0.0);

  
  // get state variables
  franka::RobotState robot_state = state_handle->getRobotState();
  Eigen::Matrix<double, 7, 1> coriolis(Eigen::Matrix<double, 7, 1>::Map(model_handle->getCoriolis().data()));
  Eigen::Matrix<double, 6, 7> jacobian(Eigen::Matrix<double, 6, 7>(model_handle->getZeroJacobian(franka::Frame::kEndEffector).data()));
  Eigen::Matrix<double, 7, 1> q(Eigen::Matrix<double, 7, 1>::Map(robot_state.q.data()));
  Eigen::Matrix<double, 7, 1> dq(Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data()));
  
  //compute target
  _desired_acceleration = _a * pole_state->get_angle() + _b * pole_state->get_dangle() + _c * franka_state->get_effector_position()(1) + _d * franka_state->get_effector_velocity()(1);
  Eigen::Matrix<double, 3, 1> position_target = franka_state->get_effector_position() + Eigen::Matrix<double, 3, 1>(0.0, _desired_acceleration, 0.0);
  Eigen::Quaterniond orientation_target(0.0, 1.0, 0.0, 0.0);

  // compute error
  Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
  error.segment<3>(0) = franka_state->get_effector_position() - position_target;
  Eigen::Quaterniond orientation = franka_state->get_effector_orientation();
  if (orientation_target.coeffs().dot(orientation.coeffs()) < 0.0) orientation.coeffs() << -orientation.coeffs();
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_target);
  error.segment<3>(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.segment<3>(3) = franka_state->get_effector_orientation().inverse() * error.segment<3>(3);

  //compute desired q
  double q_taregt_raw[] = { 0.0, 0.0, 0.0, -M_PI/2, 0.0, M_PI/2, M_PI/4 };
  Eigen::Matrix<double, 7, 1> q_target = Eigen::Matrix<double, 7, 1>::Map(q_taregt_raw);

  
  // compute control
  Eigen::Matrix<double, 7, 1> torque = coriolis;
  torque += jacobian.transpose() * (-_cartesian_stiffness * error - _cartesian_damping * (jacobian * dq));
  
  for (size_t i = 0; i < 7; ++i) joint_handles[i].setCommand(torque(i));
}

PLUGINLIB_EXPORT_CLASS(franka_pole::Controller, controller_interface::ControllerBase)