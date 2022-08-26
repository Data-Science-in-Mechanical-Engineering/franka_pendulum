#include <franka_pole/franka_model.h>
#include <franka_pole/position_controller.h>
#include <franka_pole/franka_state.h>
#include <franka_pole/parameters.h>
#include <franka_pole/pole_state.h>
#include <franka_pole/publisher.h>
#include <franka_pole/pseudo_inverse.h>

#include <type_traits>

bool franka_pole::PositionController::_init_level1(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    _position_target = parameters->target_effector_position;
    _velocity_target = Eigen::Matrix<double, 3, 1>::Zero();
    _controller_period_counter = 0;
    const double hint[] = { 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4 };
    _target_joint_positions = franka_model->effector_inverse_kinematics(parameters->initial_effector_position, parameters->initial_effector_orientation, parameters->initial_joint0_position, Eigen::Matrix<double, 7, 1>::Map(hint));
    return _init_level2(robot_hw, node_handle);
}

#ifndef FRANKA_POLE_VELOCITY_INTERFACE
Eigen::Matrix<double, 7, 1> franka_pole::PositionController::_get_torque_level1(const ros::Time &time, const ros::Duration &period)
{
    // compute target
    _controller_period_counter += parameters->command_period;
    if (_controller_period_counter >= parameters->controller_period)
    {
        _position_target = _get_position_level2(time, ros::Duration(0,1000000*parameters->controller_period));
        _velocity_target = _get_velocity_level2(time, ros::Duration(0,1000000*parameters->controller_period));
        _controller_period_counter = 0;
    }
    Eigen::Matrix<double, 7, 1> torque = Eigen::Matrix<double, 7, 1>::Zero();
    
    // apply constraints and safety
    Eigen::Matrix<double, 6, 1> cartesian_stiffness;
    Eigen::Matrix<double, 6, 1> cartesian_damping;
    bool outbound = false;
    for (size_t i = 0; i < 3; i++)
    {
        // target
        if (_position_target(i) > parameters->max_effector_position(i)) _position_target(i) = parameters->max_effector_position(i);
        else if (_position_target(i) < parameters->min_effector_position(i)) _position_target(i) = parameters->min_effector_position(i);

        if (_velocity_target(i) > parameters->max_effector_velocity(i)) _velocity_target(i) = parameters->min_effector_velocity(i);
        else if (_velocity_target(i) < parameters->min_effector_velocity(i)) _velocity_target(i) = parameters->min_effector_velocity(i);

        // real state
        bool axis_outbound = false;
        if (franka_state->get_effector_position()(i) > parameters->max_effector_position(i))
        {
            if (_velocity_target(i) > 0.0) _velocity_target(i) = 0.0;
            axis_outbound = outbound = true;
        }
        else if (franka_state->get_effector_position()(i) < parameters->min_effector_position(i))
        {
            if (_velocity_target(i) < 0.0) _velocity_target(i) = 0.0;
            axis_outbound = outbound = true;
        }

        cartesian_stiffness(i) = axis_outbound ? parameters->outbound_translation_stiffness(i) : parameters->translation_stiffness(i);
        cartesian_damping(i) = axis_outbound ? parameters->outbound_translation_damping(i) : parameters->translation_damping(i);
    }
    cartesian_stiffness.segment<3>(3) = outbound ? parameters->outbound_rotation_stiffness : parameters->rotation_stiffness;
    cartesian_damping.segment<3>(3) = outbound ? parameters->outbound_rotation_damping : parameters->rotation_damping;
    
    // calculate jacobians
    Eigen::Matrix<double, 6, 7> jacobian = franka_model->get_effector_jacobian(franka_state->get_joint_positions());
    Eigen::Matrix<double, 7, 6> jacobian_transpose;
    if (parameters->target_joint0_stuck) { jacobian_transpose.block<1,6>(0,0) = Eigen::Matrix<double, 1, 6>::Zero(); jacobian_transpose.block<6,6>(1,0) = jacobian.transpose().block<6,6>(1,0); }
    else jacobian_transpose = jacobian.transpose();
    
    Eigen::Matrix<double, 7, 6> jacobian_inverse;
    if (parameters->target_joint0_stuck) { jacobian_inverse.block<1,6>(0,0) = Eigen::Matrix<double, 1, 6>::Zero(); jacobian_inverse.block<6,6>(1,0) = pseudo_inverse(Eigen::Matrix<double, 6, 6>(jacobian.block<6,6>(0,1)), true); }
    else jacobian_inverse = pseudo_inverse(jacobian, true);
    Eigen::Matrix<double, 6, 7> jacobian_transpose_inverse = pseudo_inverse(jacobian_transpose, true);
    
    // cartesian control
    Eigen::Matrix<double, 6, 1> velocity_command = Eigen::Matrix<double, 6, 1>::Zero(); // --> to publisher
    velocity_command.segment<3>(0) = _velocity_target;
    if (!cartesian_stiffness.isZero() || !cartesian_damping.isZero())
    {
        Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
        error.segment<3>(0) = franka_state->get_effector_position() - _position_target;
        Eigen::Quaterniond orientation = franka_state->get_effector_orientation();
        if (parameters->target_effector_orientation.coeffs().dot(orientation.coeffs()) < 0.0) orientation.coeffs() = -orientation.coeffs();
        Eigen::Quaterniond error_quaternion(orientation.inverse() * parameters->target_effector_orientation);
        error.segment<3>(3) = -(franka_state->get_effector_orientation() * Eigen::Matrix<double, 3, 1>(error_quaternion.x(), error_quaternion.y(), error_quaternion.z()));
        torque += jacobian_transpose * (-cartesian_stiffness.array() * error.array() - cartesian_damping.array() * (franka_state->get_effector_velocity() - velocity_command).array()).matrix();
    }
    
    // joints-space control
    Eigen::Matrix<double, 7, 1> joint_position_command = Eigen::Matrix<double, 7, 1>::Zero(); //--> to publisher
    Eigen::Matrix<double, 7, 1> joint_velocity_command = Eigen::Matrix<double, 7, 1>::Zero(); //--> to publisher
    if (!parameters->joint_stiffness.isZero() || !parameters->joint_damping.isZero())
    {
        try
        {
            joint_position_command = franka_model->effector_inverse_kinematics(_position_target, parameters->target_effector_orientation, parameters->target_joint0_stuck ? parameters->target_joint0_position : std::numeric_limits<double>::quiet_NaN(), _target_joint_positions);
            joint_velocity_command = jacobian_inverse.block<7,3>(0,0) * _velocity_target;
            torque += (
                parameters->joint_stiffness.array() * (joint_position_command - franka_state->get_joint_positions()).array() +
                parameters->joint_damping.array() * (joint_velocity_command - franka_state->get_joint_velocities()).array()
            ).matrix();
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
        }
    }
    
    // nullspace control
    if (!parameters->nullspace_stiffness.isZero() || !parameters->nullspace_damping.isZero())
    {
        torque += (Eigen::Matrix<double, 7, 7>::Identity() - jacobian_transpose * jacobian_transpose_inverse) * (
            parameters->nullspace_stiffness.array() * (_target_joint_positions - franka_state->get_joint_positions()).array() +
            parameters->nullspace_damping.array() * (/*zero velocity*/ - franka_state->get_joint_velocities()).array()
        ).matrix();
    }
    
    // gravit and coriolis
    if (get_model_freedom(parameters->model) == 0)
    {
        Eigen::Matrix<double, 9, 1> gravity = franka_model->get_gravity9(franka_state->get_joint_positions());
        Eigen::Matrix<double, 9, 1> coriolis = franka_model->get_coriolis9(franka_state->get_joint_positions(), franka_state->get_joint_velocities());
        torque += coriolis.segment<7>(0) + gravity.segment<7>(0);
    }
    else if (get_model_freedom(parameters->model) == 1)
    {
        Eigen::Matrix<double, 10, 1> gravity = franka_model->get_gravity10(franka_state->get_joint_positions(), pole_state->get_joint_angle());
        Eigen::Matrix<double, 10, 1> coriolis = franka_model->get_coriolis10(franka_state->get_joint_positions(), franka_state->get_joint_velocities(), pole_state->get_joint_angle(), pole_state->get_joint_dangle());
        torque += coriolis.segment<7>(0) + gravity.segment<7>(0);
    }
    else
    {
        Eigen::Matrix<double, 11, 1> gravity = franka_model->get_gravity11(franka_state->get_joint_positions(), pole_state->get_joint_angle());
        Eigen::Matrix<double, 11, 1> coriolis = franka_model->get_coriolis11(franka_state->get_joint_positions(), franka_state->get_joint_velocities(), pole_state->get_joint_angle(), pole_state->get_joint_dangle());
        torque += coriolis.segment<7>(0) + gravity.segment<7>(0);
    }
    
    // publish
    publisher->set_command(
        time,
        _position_target,
        parameters->target_effector_orientation,
        velocity_command,
        Eigen::Matrix<double, 6, 1>::Zero(),
        joint_position_command,
        joint_velocity_command,
        Eigen::Matrix<double, 7, 1>::Zero(),
        torque);

    return torque;
}
#else
Eigen::Matrix<double, 6, 1> franka_pole::PositionController::_get_velocity_level1(const ros::Time &time, const ros::Duration &period)
{
    // compute target
    _controller_period_counter += parameters->command_period;
    if (_controller_period_counter >= parameters->controller_period)
    {
        _position_target = _get_position_level2(time, ros::Duration(0,1000000*parameters->controller_period));
        _velocity_target = _get_velocity_level2(time, ros::Duration(0,1000000*parameters->controller_period));
        _controller_period_counter = 0;
    }
    
    // apply constraints and safety
    for (size_t i = 0; i < 3; i++)
    {
        if (_velocity_target(i) > parameters->max_effector_velocity(i)) _velocity_target(i) = parameters->min_effector_velocity(i);
        else if (_velocity_target(i) < parameters->min_effector_velocity(i)) _velocity_target(i) = parameters->min_effector_velocity(i);
    }
    
    // publish
    Eigen::Matrix<double, 6, 1> velocity_command = Eigen::Matrix<double, 6, 1>::Zero();
    velocity_command.segment<3>(0) = _velocity_target;
    publisher->set_command(
        time,
        _position_target,
        parameters->target_effector_orientation,
        velocity_command,
        Eigen::Matrix<double, 6, 1>::Zero(),
        Eigen::Matrix<double, 7, 1>::Zero(),
        Eigen::Matrix<double, 7, 1>::Zero(),
        Eigen::Matrix<double, 7, 1>::Zero(),
        Eigen::Matrix<double, 7, 1>::Zero());

    return velocity_command;
}
#endif