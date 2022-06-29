#include <franka_pole/position_controller.h>
#include <franka_pole/franka_state.h>
#include <franka_pole/pole_state.h>
#include <franka_pole/publisher.h>
#include <franka_pole/pseudo_inverse.h>

bool franka_pole::PositionController::_controller_init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    if (!Controller::_controller_init(robot_hw, node_handle)) return false;

    _cartesian_stiffness.setZero();
    _cartesian_stiffness.diagonal().segment<3>(0) = get_translation_stiffness();
    _cartesian_stiffness.diagonal().segment<3>(3) = get_rotation_stiffness();
    _cartesian_damping = 2.0 * _cartesian_stiffness.array().sqrt().matrix();

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
    publisher->set_command_timestamp(time);
    publisher->set_command_effector_position(position_target);
    publisher->set_command_effector_velocity(velocity_target);
    publisher->set_command_effector_acceleration(Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0));

    // conventional control:
    // compute target
    Eigen::Quaterniond orientation_target(0.0, 1.0, 0.0, 0.0);
    Eigen::Matrix<double, 6, 1> velocity_target6 = Eigen::Matrix<double, 6, 1>::Zero();
    velocity_target6.segment<3>(0) = velocity_target;

    // compute error
    Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
    error.segment<3>(0) = franka_state->get_effector_position() - position_target;
    Eigen::Quaterniond orientation = franka_state->get_effector_orientation();
    if (orientation_target.coeffs().dot(orientation.coeffs()) < 0.0) orientation.coeffs() << -orientation.coeffs();
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_target);
    error.segment<3>(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error.segment<3>(3) = -franka_state->get_effector_transform().linear() * error.segment<3>(3);

    // compute torque
    Eigen::Matrix<double, 7, 1> torque = franka_model->get_coriolis(franka_state->get_joint_positions(), franka_state->get_joint_velocities(), pole_state->get_angle(), pole_state->get_dangle());
    Eigen::Matrix<double, 6, 7> jacobian = franka_model->get_effector_jacobian(franka_state->get_joint_positions());
    Eigen::Matrix<double, 7, 6> jacobian_transpose = jacobian.transpose();
    torque += jacobian_transpose * (-_cartesian_stiffness * error - _cartesian_damping * (franka_state->get_effector_velocity() - velocity_target6));
    torque += (Eigen::Matrix<double, 7, 7>::Identity() - jacobian_transpose * pseudo_inverse(jacobian_transpose, true)) *
    (get_nullspace_stiffness().array() * (get_initial_joint_positions().segment<7>(0) - franka_state->get_joint_positions()).array() - 2 * get_nullspace_stiffness().array().sqrt() * franka_state->get_joint_velocities().array()).matrix();

    Controller::_controller_post_update(time, period, torque);
}