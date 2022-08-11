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
    _target_joint_positions = franka_model->effector_inverse_kinematics(parameters->initial_effector_position, parameters->initial_effector_orientation, parameters->initial_joint0_position);
    return _init_level2(robot_hw, node_handle);
}

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
        if (_position_target(i) > parameters->max_effector_position(i))
        {
            _position_target(i) = parameters->max_effector_position(i);
        }
        else if (_position_target(i) < parameters->min_effector_position(i))
        {
            _position_target(i) = parameters->min_effector_position(i);
        }

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
    Eigen::Matrix<double, 7, 6> jacobian_transpose = jacobian.transpose();
    Eigen::Matrix<double, 7, 6> jacobian_inverse = pseudo_inverse(jacobian, true);
    Eigen::Matrix<double, 6, 7> jacobian_transpose_inverse = pseudo_inverse(jacobian_transpose, true);
    
    // cartesian control
    if (!cartesian_stiffness.isZero() || !cartesian_damping.isZero())
    {
        Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
        error.segment<3>(0) = franka_state->get_effector_position() - _position_target;
        Eigen::Quaterniond orientation = franka_state->get_effector_orientation();
        if (parameters->target_effector_orientation.coeffs().dot(orientation.coeffs()) < 0.0) orientation.coeffs() = -orientation.coeffs();
        Eigen::Quaterniond error_quaternion(orientation.inverse() * parameters->target_effector_orientation);
        error.segment<3>(3) = -(franka_state->get_effector_orientation() * Eigen::Matrix<double, 3, 1>(error_quaternion.x(), error_quaternion.y(), error_quaternion.z()));
        Eigen::Matrix<double, 6, 1> v6 = Eigen::Matrix<double, 6, 1>::Zero();
        v6.segment<3>(0) = _velocity_target;
        torque += jacobian_transpose * (-cartesian_stiffness.array() * error.array() - cartesian_damping.array() * (franka_state->get_effector_velocity() - v6).array()).matrix();
    }
    
    // joints-space control
    if (!parameters->joint_stiffness.isZero() || !parameters->joint_damping.isZero())
    {
        try
        {
            Eigen::Matrix<double, 7, 1> joint_position_target = franka_model->effector_inverse_kinematics(_position_target, parameters->target_effector_orientation, std::numeric_limits<double>::quiet_NaN());
            Eigen::Matrix<double, 7, 1> joint_velocity_target = jacobian_inverse.block<7,3>(0,0) * _velocity_target;
            torque += (
                parameters->joint_stiffness.array() * (franka_state->get_joint_positions() - joint_position_target).array() +
                parameters->joint_damping.array() * (franka_state->get_joint_velocities() - joint_velocity_target).array()
            ).matrix();
        }
        catch (std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
        }
    }
    
    // nullspace control
    if (!parameters->nullspace_stiffness.isZero() || !parameters->nullspace_damping.isZero())
    {
        torque += (Eigen::Matrix<double, 7, 7>::Identity() - jacobian_transpose * jacobian_transpose_inverse) * (
            parameters->nullspace_stiffness.array() * (_target_joint_positions - franka_state->get_joint_positions()).array() +
            parameters->nullspace_damping.array() * (-franka_state->get_joint_velocities()).array()
        ).matrix();
    }

    // anti-damping
    if (parameters->simulated) torque += franka_state->get_joint_velocities() * 0.003;

    // gravit and coriolis
    if (parameters->model == Model::D0)
    {
        Eigen::Matrix<double, 9, 1> gravity = franka_model->get_gravity9(franka_state->get_joint_positions());
        Eigen::Matrix<double, 9, 1> coriolis = franka_model->get_coriolis9(franka_state->get_joint_positions(), franka_state->get_joint_velocities());
        torque += coriolis.segment<7>(0) + gravity.segment<7>(0);
    }
    else if (parameters->model == Model::D1)
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
    publisher->set_command_timestamp(time);
    publisher->set_command_effector_position(_position_target);
    publisher->set_command_effector_velocity(_velocity_target);
    publisher->set_command_effector_acceleration(Eigen::Matrix<double, 3, 1>::Zero());
    publisher->set_command_joint_torques(torque);

    return torque;
}