#include <franka_pole/acceleration_controller.h>
#include <franka_pole/franka_state.h>
#include <franka_pole/pole_state.h>
#include <franka_pole/publisher.h>
#include <franka_pole/pseudo_inverse.h>

#include <ros/package.h>

bool franka_pole::AccelerationController::_controller_init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    if (!Controller::_controller_init(robot_hw, node_handle)) return false;

    _cartesian_stiffness.setZero();
    _cartesian_stiffness.diagonal().segment<3>(0) = param->translation_stiffness();
    _cartesian_stiffness.diagonal().segment<3>(3) = param->rotation_stiffness();
    _cartesian_damping = 2.0 * _cartesian_stiffness.array().sqrt().matrix();
    
    return true;
}

void franka_pole::AccelerationController::_controller_starting(const ros::Time &time)
{
    Controller::_controller_starting(time);
}

void franka_pole::AccelerationController::_controller_pre_update(const ros::Time &time, const ros::Duration &period)
{
    Controller::_controller_pre_update(time, period);
}

void franka_pole::AccelerationController::_controller_post_update(const ros::Time &time, const ros::Duration &period, const Eigen::Matrix<double, 3, 1> &acceleration_target)
{
    // Conventional controller:
    // compute target
    Eigen::Vector3d position_target = param->box_center();
    Eigen::Quaterniond orientation_target(0.0, 1.0, 0.0, 0.0);
    if (std::max(param->box_min()(1), std::min(franka_state->get_effector_position()(1), param->box_max()(1))) != franka_state->get_effector_position()(1))
    {
        position_target(1) = std::max(param->box_min()(1), std::min(franka_state->get_effector_position()(1), param->box_max()(1))); //We care about Y
        _cartesian_stiffness.diagonal()(1) = param->translation_stiffness()(1);
        _cartesian_damping.diagonal()(1) = 2 * sqrt(param->translation_stiffness()(1));
    }
    else
    {
        _cartesian_stiffness.diagonal()(1) = 0.0; //We don't care about Y
        _cartesian_damping.diagonal()(1) = 0.0;
    }
    if (param->two_dimensional())
    {
        if (std::max(param->box_min()(0), std::min(franka_state->get_effector_position()(0), param->box_max()(0))) != franka_state->get_effector_position()(0))
        {
            position_target(0) = std::max(param->box_min()(0), std::min(franka_state->get_effector_position()(0), param->box_max()(0))); //We care about X
            _cartesian_stiffness.diagonal()(0) = param->translation_stiffness()(0);
            _cartesian_damping.diagonal()(0) = 2 * sqrt(param->translation_stiffness()(0));
        }
        else
        {
            _cartesian_stiffness.diagonal()(0) = 0.0; //We don't care about X
            _cartesian_damping.diagonal()(0) = 0.0;
        }
    }

    
    // compute error
    Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
    error.segment<3>(0) = franka_state->get_effector_position() - position_target;
    Eigen::Quaterniond orientation = franka_state->get_effector_orientation();
    if (orientation_target.coeffs().dot(orientation.coeffs()) < 0.0) orientation.coeffs() << -orientation.coeffs();
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_target);
    error.segment<3>(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error.segment<3>(3) = -franka_state->get_effector_transform().linear() * error.segment<3>(3);

    // compute torque
    Eigen::Matrix<double, 7, 1> torque = franka_state->get_coriolis();
    Eigen::Matrix<double, 6, 7> jacobian = franka_state->get_effector_jacobian();
    Eigen::Matrix<double, 7, 6> jacobian_transpose = jacobian.transpose();
    torque += jacobian_transpose * (-_cartesian_stiffness * error - _cartesian_damping * franka_state->get_effector_velocity());
    torque += (Eigen::Matrix<double, 7, 7>::Identity() - jacobian_transpose * pseudo_inverse(jacobian_transpose, true)) *
    (param->nullspace_stiffness().array() * (param->initial_joint_positions().segment<7>(0) - franka_state->get_joint_positions()).array() - 2 * param->nullspace_stiffness().array().sqrt() * franka_state->get_joint_velocities().array()).matrix();

    // acceleration control
    torque += franka_model->get_torques(franka_state->get_joint_positions(), franka_state->get_joint_velocities(), pole_state->get_angle(), pole_state->get_dangle(), acceleration_target);

    // publish
    publisher->set_command_timestamp(time);
    publisher->set_command_effector_position(position_target);
    publisher->set_command_effector_velocity(Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0));
    publisher->set_command_effector_acceleration(acceleration_target);

    Controller::_controller_post_update(time, period, torque);
}