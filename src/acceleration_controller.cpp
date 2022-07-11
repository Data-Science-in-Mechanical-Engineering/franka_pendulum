#include <franka_pole/acceleration_controller.h>
#include <franka_pole/franka_state.h>
#include <franka_pole/parameters.h>
#include <franka_pole/pole_state.h>
#include <franka_pole/publisher.h>
#include <franka_pole/pseudo_inverse.h>

#include <ros/package.h>

bool franka_pole::AccelerationController::_controller_init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    if (!Controller::_controller_init(robot_hw, node_handle)) return false;

    Parameters parameters(node_handle);
    _two_dimensional = parameters.two_dimensional();
    _cartesian_stiffness.segment<3>(0) = parameters.translation_stiffness();
    _cartesian_stiffness.segment<3>(3) = parameters.rotation_stiffness();
    _cartesian_damping = 2.0 * _cartesian_stiffness.array().sqrt().matrix();
    _cartesian_stiffness_safety = parameters.translation_stiffness_safety();
    _cartesian_damping_safety = 2.0 * _cartesian_stiffness_safety.array().sqrt().matrix();
    _nullspace_stiffness = parameters.nullspace_stiffness();
    _position_target = parameters.target_effector_position();
    _orientation_target = parameters.target_effector_orientation();
    _max_effector_position = parameters.max_effector_position();
    _min_effector_position = parameters.min_effector_position();
    _target_joint_positions = franka_model->inverse_kinematics(parameters.target_effector_position(), parameters.target_effector_orientation(), parameters.initial_joint0_position());

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
    Eigen::Matrix<double, 3, 1> position_target = Eigen::Matrix<double, 3, 1>::Zero();
    Eigen::Matrix<double, 6, 6> cartesian_stiffness = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> cartesian_damping = Eigen::Matrix<double, 6, 6>::Zero();
    cartesian_stiffness.diagonal().segment<3>(3) = _cartesian_stiffness.segment<3>(3); //Always care about rotation
    cartesian_damping.diagonal().segment<3>(3) = _cartesian_damping.segment<3>(3);

    //set boundaries
    for (size_t i = 0; i < 3; i++)
    {
        double clipped = std::max(_min_effector_position(i), std::min(franka_state->get_effector_position()(i), _max_effector_position(i)));
        if (clipped != franka_state->get_effector_position()(i)) //Outbounds
        {
            position_target(i) = clipped;
            cartesian_stiffness(i,i) = _cartesian_stiffness_safety(i);
            cartesian_damping(i,i) = _cartesian_damping_safety(i);
        }
        else //Inbounds
        {
            position_target(i) = _position_target(i);
            cartesian_stiffness(i,i) = _cartesian_stiffness(i);
            cartesian_damping(i,i) = _cartesian_damping(i);
        }
    }
    
    // compute error
    Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
    error.segment<3>(0) = franka_state->get_effector_position() - position_target;
    Eigen::Quaterniond orientation = franka_state->get_effector_orientation();
    if (_orientation_target.coeffs().dot(orientation.coeffs()) < 0.0) orientation.coeffs() << -orientation.coeffs();
    Eigen::Quaterniond error_quaternion(orientation.inverse() * _orientation_target);
    error.segment<3>(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error.segment<3>(3) = -(franka_state->get_effector_orientation() * error.segment<3>(3));

    // compute torque
    Eigen::Matrix<double, 6, 7> jacobian = franka_model->get_effector_jacobian(franka_state->get_joint_positions());
    Eigen::Matrix<double, 7, 6> jacobian_transpose = jacobian.transpose();
    Eigen::Matrix<double, 7, 1> torque = Eigen::Matrix<double, 7, 1>::Zero();
    torque += franka_model->get_gravity(franka_state->get_joint_positions(), pole_state->get_joint_angle());
    torque += franka_model->get_coriolis(franka_state->get_joint_positions(), franka_state->get_joint_velocities(), pole_state->get_joint_angle(), pole_state->get_joint_dangle());
    torque += jacobian_transpose * (-cartesian_stiffness * error - cartesian_damping * franka_state->get_effector_velocity());
    torque += (Eigen::Matrix<double, 7, 7>::Identity() - jacobian_transpose * pseudo_inverse(jacobian_transpose, true)) *
    (_nullspace_stiffness.array() * (_target_joint_positions - franka_state->get_joint_positions()).array() - 2 * _nullspace_stiffness.array().sqrt() * franka_state->get_joint_velocities().array()).matrix();

    // acceleration control
    Eigen::Matrix<double, 6, 1> a6 = Eigen::Matrix<double, 6, 1>::Zero();
    a6.segment<3>(0) = acceleration_target;
    torque += franka_model->get_torques(franka_state->get_joint_positions(), franka_state->get_joint_velocities(), pole_state->get_joint_angle(), pole_state->get_joint_dangle(), a6);

    // publish
    publisher->set_command_effector_position(position_target);
    publisher->set_command_effector_velocity(Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0));
    publisher->set_command_effector_acceleration(acceleration_target);

    Controller::_controller_post_update(time, period, torque);
}