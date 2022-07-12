#include <pinocchio/fwd.hpp>
#include <franka_pole/position_controller.h>
#include <franka_pole/franka_state.h>
#include <franka_pole/parameters.h>
#include <franka_pole/pole_state.h>
#include <franka_pole/publisher.h>
#include <franka_pole/pseudo_inverse.h>

bool franka_pole::PositionController::_controller_init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    if (!Controller::_controller_init(robot_hw, node_handle)) return false;
    
    Parameters parameters(node_handle);
    _cartesian_stiffness.setZero();
    _cartesian_stiffness.diagonal().segment<3>(0) = parameters.translation_stiffness();
    _cartesian_stiffness.diagonal().segment<3>(3) = parameters.rotation_stiffness();
    _cartesian_damping = 2.0 * _cartesian_stiffness.array().sqrt().matrix();
    _nullspace_stiffness = parameters.nullspace_stiffness();
    _orientation_target = parameters.target_effector_orientation();
    _target_joint_positions = franka_model->inverse_kinematics(parameters.target_effector_position(), parameters.target_effector_orientation(), parameters.initial_joint0_position());

    return true;
}

void franka_pole::PositionController::_controller_starting(const ros::Time &time)
{
    Controller::_controller_starting(time);
}

void franka_pole::PositionController::_controller_pre_update(const ros::Time &time, const ros::Duration &period)
{
    Controller::_controller_pre_update(time, period);
}

void franka_pole::PositionController::_controller_post_update(const ros::Time &time, const ros::Duration &period, const Eigen::Matrix<double, 3, 1> &position_target, const Eigen::Matrix<double, 3, 1> &velocity_target)
{
    Controller::_controller_pre_update(time, period);

    // publish
    publisher->set_command_effector_position(position_target);
    publisher->set_command_effector_velocity(velocity_target);
    publisher->set_command_effector_acceleration(Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0));

    // basics
    Eigen::Matrix<double, 7, 1> torque = Eigen::Matrix<double, 7, 1>::Zero();
    torque += franka_model->get_gravity(franka_state->get_joint_positions(), pole_state->get_joint_angle());
    Eigen::Matrix<double, 6, 1> v6 = Eigen::Matrix<double, 6, 1>::Zero();
    v6.segment<3>(0) = velocity_target;
    torque += franka_model->get_coriolis(franka_state->get_joint_positions(), franka_state->get_joint_velocities(), pole_state->get_joint_angle(), pole_state->get_joint_dangle());
    
    // conventional control
    Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
    error.segment<3>(0) = franka_state->get_effector_position() - position_target;
    Eigen::Quaterniond orientation = franka_state->get_effector_orientation();
    if (_orientation_target.coeffs().dot(orientation.coeffs()) < 0.0) orientation.coeffs() << -orientation.coeffs();
    Eigen::Quaterniond error_quaternion(orientation.inverse() * _orientation_target);
    error.segment<3>(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error.segment<3>(3) = -(franka_state->get_effector_orientation() * error.segment<3>(3));
    Eigen::Matrix<double, 6, 7> jacobian = franka_model->get_effector_jacobian(franka_state->get_joint_positions());
    Eigen::Matrix<double, 7, 6> jacobian_transpose = jacobian.transpose();
    torque += jacobian_transpose * (-_cartesian_stiffness * error - _cartesian_damping * (franka_state->get_effector_velocity() - v6));

    // nullspace control
    torque += (Eigen::Matrix<double, 7, 7>::Identity() - jacobian_transpose * pseudo_inverse(jacobian_transpose, true)) *
    (_nullspace_stiffness.array() * (_target_joint_positions - franka_state->get_joint_positions()).array() - 2 * _nullspace_stiffness.array().sqrt() * franka_state->get_joint_velocities().array()).matrix();

    // anti-damping
    torque += franka_state->get_joint_velocities() * 0.003;

    Controller::_controller_post_update(time, period, torque);
}