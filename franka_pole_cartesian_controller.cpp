#include "franka_pole_cartesian_controller.h"

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

bool franka_pole::CartesianController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
  _cartesian_target_subscriber = node_handle.subscribe("/franka_pole/cartesian_target", 20, &CartesianController::_cartesian_target_callback, this, ros::TransportHints().reliable().tcpNoDelay());

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
    _model_handle = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
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
    _state_handle = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
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
      _joint_handles.push_back(effort_joint_interface->getHandle(joint_names[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("CartesianController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  _cartesian_stiffness.setZero();
  _cartesian_stiffness.diagonal().segment<3>(0) = Eigen::Matrix<double, 3, 1>::Ones() * 200.0;
  _cartesian_stiffness.diagonal().segment<3>(3) = Eigen::Matrix<double, 3, 1>::Ones() * 10.0;
  _nullspace_stiffness = 0.5;
  _nullspace_damping = 2.0 * sqrt(_nullspace_stiffness);
  _cartesian_damping = 2.0 * _cartesian_stiffness.array().sqrt().matrix();
  return true;
}

void franka_pole::CartesianController::starting(const ros::Time &)
{
  franka::RobotState initial_state = _state_handle->getRobotState();
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  _position_target = initial_transform.translation();
  _orientation_target = Eigen::Quaterniond(initial_transform.linear());
}

void franka_pole::CartesianController::update(const ros::Time &, const ros::Duration &)
{
  // get state variables
  franka::RobotState robot_state = _state_handle->getRobotState();
  Eigen::Matrix<double, 7, 1> coriolis(Eigen::Matrix<double, 7, 1>::Map(_model_handle->getCoriolis().data()));
  Eigen::Matrix<double, 6, 7> jacobian(Eigen::Matrix<double, 6, 7>(_model_handle->getZeroJacobian(franka::Frame::kEndEffector).data()));
  Eigen::Matrix<double, 7, 1> q(Eigen::Matrix<double, 7, 1>::Map(robot_state.q.data()));
  Eigen::Matrix<double, 7, 1> dq(Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data()));
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  // compute perfect q
  double q_target_raw [] = { 0.0, 0.0, 0.0, -M_PI/2, 0.0, M_PI/2, M_PI/4 };
  Eigen::Matrix<double, 7, 1> q_target = Eigen::Matrix<double, 7, 1>::Map(q_target_raw);
  
  // compute error
  Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
  error.segment<3>(0) = position - _position_target;
  if (_orientation_target.coeffs().dot(orientation.coeffs()) < 0.0) orientation.coeffs() << -orientation.coeffs();
  Eigen::Quaterniond error_quaternion(orientation.inverse() * _orientation_target);
  error.segment<3>(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.segment<3>(3) = -transform.linear() * error.segment<3>(3);

  // compute control
  Eigen::Matrix<double, 7, 1> torque = coriolis;
  torque += jacobian.transpose() * (-_cartesian_stiffness * error - _cartesian_damping * (jacobian * dq));
  Eigen::Matrix<double, 6, 7> jacobian_transpose_pinverse = _pseudo_inverse(Eigen::Matrix<double, 7, 6>(jacobian.transpose()), true);
  torque += (Eigen::Matrix<double, 7, 7>::Identity() - jacobian.transpose() * jacobian_transpose_pinverse) * (_nullspace_stiffness * (q_target - q) - _nullspace_damping * dq);
  
  for (size_t i = 0; i < 7; ++i) _joint_handles[i].setCommand(torque(i));
}

template <int H, int W> Eigen::Matrix<double, W, H> franka_pole::CartesianController::_pseudo_inverse(const Eigen::Matrix<double, H, W> &matrix, bool damped)
{
  double lambda = damped ? 0.2 : 0.0;
  Eigen::JacobiSVD<Eigen::Matrix<double, H, W>> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  typename Eigen::JacobiSVD<Eigen::Matrix<double, W, W>>::SingularValuesType singular_values = svd.singularValues();
  Eigen::Matrix<double, H, W> s = Eigen::Matrix<double, H, W>::Zero();
  for (unsigned int i = 0; i < singular_values.size(); i++) s(i, i) = (singular_values(i)) / (singular_values(i) * singular_values(i) + lambda * lambda);
  return svd.matrixV() * s.transpose() * svd.matrixU().transpose();
}

void franka_pole::CartesianController::_cartesian_target_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  _position_target << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  _orientation_target.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
}

PLUGINLIB_EXPORT_CLASS(franka_pole::CartesianController, controller_interface::ControllerBase)