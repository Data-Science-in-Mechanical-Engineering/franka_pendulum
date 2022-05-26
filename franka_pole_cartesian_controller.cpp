#include "franka_pole_cartesian_controller.h"

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

//================================================================== Utilities ==================================================================

template <int H, int W> Eigen::Matrix<double, W, H> pseudo_inverse(const Eigen::Matrix<double, H, W> &matrix, bool damped)
{
  double lambda = damped ? 0.2 : 0.0;
  Eigen::JacobiSVD<Eigen::Matrix<double, H, W>> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  typename Eigen::JacobiSVD<Eigen::Matrix<double, W, W>>::SingularValuesType singular_values = svd.singularValues();
  Eigen::Matrix<double, H, W> s = Eigen::Matrix<double, H, W>::Zero();
  for (unsigned int i = 0; i < singular_values.size(); i++) s(i, i) = (singular_values(i)) / (singular_values(i) * singular_values(i) + lambda * lambda);
  return svd.matrixV() * s.transpose() * svd.matrixU().transpose();
}

//================================================================== FrankaState ==================================================================

franka_pole::FrankaState::FrankaState(CartesianController *controller, ros::NodeHandle &)
{
  _controller = controller;
}

void franka_pole::FrankaState::update(double time)
{
  _time = time;

  franka::RobotState robot_state = _controller->state_handle->getRobotState();

  _joint_positions = Eigen::Matrix<double, 7, 1>::Map(robot_state.q.data());
  _joint_velocities = Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data());
  _coriolis = Eigen::Matrix<double, 7, 1>::Map(_controller->model_handle->getCoriolis().data());

  _effector_jacobian = Eigen::Matrix<double, 6, 7>::Map(_controller->model_handle->getZeroJacobian(franka::Frame::kEndEffector).data());
  _effector_transform = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
  _effector_position = _effector_transform.translation();
  _effector_velocity = (_effector_jacobian * _joint_velocities);
  _effector_orientation = Eigen::Quaterniond(_effector_transform.linear());
}

double franka_pole::FrankaState::get_time()
{
  return _time;
}

Eigen::Vector3d franka_pole::FrankaState::get_effector_position()
{
  return _effector_position;
}

Eigen::Matrix<double, 6, 1> franka_pole::FrankaState::get_effector_velocity()
{
  return _effector_velocity;
}

Eigen::Quaterniond franka_pole::FrankaState::get_effector_orientation()
{
  return _effector_orientation;
}

Eigen::Affine3d franka_pole::FrankaState::get_effector_transform()
{
  return _effector_transform;
}

Eigen::Matrix<double, 6, 7> franka_pole::FrankaState::get_effector_jacobian()
{
  return _effector_jacobian;
}

Eigen::Matrix<double, 7, 1> franka_pole::FrankaState::get_joint_positions()
{
  return _joint_positions;
}

Eigen::Matrix<double, 7, 1> franka_pole::FrankaState::get_joint_velocities()
{
  return _joint_velocities;
}

Eigen::Matrix<double, 7, 1> franka_pole::FrankaState::get_coriolis()
{
  return _coriolis;
}

//================================================================== PoleState ==================================================================

void franka_pole::PoleState::_update(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  //todo
}

franka_pole::PoleState::PoleState(CartesianController *controller, ros::NodeHandle &node_handle)
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

franka_pole::Sampler::Sampler(CartesianController *controller, ros::NodeHandle &node_handle)
{
  _controller = controller;
  _debug_sample_publisher = node_handle.advertise<franka_pole::DebugSample>("/franka_pole/samples", 20);
  _debug_sample_timer = node_handle.createTimer(ros::Duration(0.01), &franka_pole::Sampler::_debug_sample_callback, this);
}

//================================================================== CartesianController ==================================================================

bool franka_pole::CartesianController::is_simulated()
{
  return _simulated;
}

franka_pole::ControlType franka_pole::CartesianController::control_type()
{
  return _control_type;
}

double franka_pole::CartesianController::get_desired_acceleration()
{
  return _desired_acceleration;
}

bool franka_pole::CartesianController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id))
  {
    ROS_ERROR_STREAM("CartesianController: Could not read parameter arm_id");
    return false;
  }
  
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
  {
    ROS_ERROR("CartesianController: Invalid or no joint_names parameters provided, aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr)
  {
    ROS_ERROR_STREAM("CartesianController: Error getting model interface from hardware");
    return false;
  }
  try
  {
    model_handle = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
  }
  catch (hardware_interface::HardwareInterfaceException& ex)
  {
    ROS_ERROR_STREAM("CartesianController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr)
  {
    ROS_ERROR_STREAM("CartesianController: Error getting state interface from hardware");
    return false;
  }
  try
  {
    state_handle = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
  }
  catch (hardware_interface::HardwareInterfaceException &ex)
  {
    ROS_ERROR_STREAM("CartesianController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr)
  {
    ROS_ERROR_STREAM("CartesianController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i)
  {
    try
    {
      joint_handles.push_back(effort_joint_interface->getHandle(joint_names[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("CartesianController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  auto* position_joint_interface = robot_hw->get<hardware_interface::PositionJointInterface>();
  try
  {
    joint_handles.push_back(position_joint_interface->getHandle("panda_lower_upper"));
  }
  catch (const hardware_interface::HardwareInterfaceException &ex)
  {
    ROS_ERROR_STREAM("CartesianController: Exception getting joint handles: " << ex.what());
    return false;
  }

  _cartesian_stiffness.setZero();
  _cartesian_stiffness.diagonal().segment<3>(0) = Eigen::Vector3d::Ones() * 200.0;
  _cartesian_stiffness.diagonal().segment<3>(3) = Eigen::Vector3d::Ones() * 10.0;
  _nullspace_stiffness = 0.5;
  _nullspace_damping = 2.0 * sqrt(_nullspace_stiffness);
  _cartesian_damping = 2.0 * _cartesian_stiffness.array().sqrt().matrix();

  franka_state = std::make_unique<FrankaState>(this, node_handle);
  pole_state  = std::make_unique<PoleState>(this, node_handle);
  sampler  = std::make_unique<Sampler>(this, node_handle);
  return true;
}

void franka_pole::CartesianController::starting(const ros::Time &)
{
}

void franka_pole::CartesianController::update(const ros::Time &, const ros::Duration &)
{
  franka_state->update(0.0);
  pole_state->update(0.0);

  //compute target
  _desired_acceleration = -_a * pole_state->get_angle() + -_b * pole_state->get_dangle() + _c * franka_state->get_effector_position()(1) + _d * franka_state->get_effector_velocity()(1);
  Eigen::Vector3d position_target(0.5, std::max(-0.6, std::min(franka_state->get_effector_position()(1) + _desired_acceleration, 0.6)), 0.5);
  Eigen::Quaterniond orientation_target(0.0, 1.0, 0.0, 0.0);

  // compute error
  Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
  error.segment<3>(0) = franka_state->get_effector_position() - position_target;
  Eigen::Quaterniond orientation = franka_state->get_effector_orientation();
  if (orientation_target.coeffs().dot(orientation.coeffs()) < 0.0) orientation.coeffs() << -orientation.coeffs();
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_target);
  error.segment<3>(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.segment<3>(3) = -franka_state->get_effector_transform().linear() * error.segment<3>(3);

  //compute desired q
  double q_taregt_raw[] = { 0.0, 0.0, 0.0, -M_PI/2, 0.0, M_PI/2, M_PI/4 };
  Eigen::Matrix<double, 7, 1> q_target = Eigen::Matrix<double, 7, 1>::Map(q_taregt_raw);

  // compute control
  Eigen::Matrix<double, 7, 1> torque = franka_state->get_coriolis();
  Eigen::Matrix<double, 7, 6> jacobian_transpose = franka_state->get_effector_jacobian().transpose();
  torque += jacobian_transpose * (-_cartesian_stiffness * error - _cartesian_damping * franka_state->get_effector_velocity());
  torque += (Eigen::Matrix<double, 7, 7>::Identity() - jacobian_transpose * pseudo_inverse(jacobian_transpose, true)) *
    (_nullspace_stiffness * (q_target - franka_state->get_joint_positions()) - _nullspace_damping * franka_state->get_joint_velocities());

  
  for (size_t i = 0; i < 7; ++i) joint_handles[i].setCommand(torque(i));
}

PLUGINLIB_EXPORT_CLASS(franka_pole::CartesianController, controller_interface::ControllerBase)