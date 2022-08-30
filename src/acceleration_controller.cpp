#include <franka_pole/franka_model.h>
#include <franka_pole/acceleration_controller.h>
#include <franka_pole/franka_state.h>
#include <franka_pole/parameters.h>
#include <franka_pole/pole_state.h>
#include <franka_pole/publisher.h>
#include <franka_pole/pseudo_inverse.h>

#include <type_traits>

void franka_pole::AccelerationController::_update_cartesian_targets(const ros::Time &time, const ros::Duration &period)
{
    // get acceleration
    _controller_period_counter += parameters->command_period;
    if (_controller_period_counter >= parameters->controller_period)
    {
        _input_acceleration = _get_acceleration_level2(time, ros::Duration(0,1000000*parameters->controller_period));
        _controller_period_counter = 0;
    }

    // integrate
    _velocity_target.segment<3>(0) += period.toSec() * _input_acceleration;
    _position_target += period.toSec() * _velocity_target.segment<3>(0);

    // apply constraints
    for (size_t i = 0; i < 3; i++)
    {
        // target
        if (_position_target(i) > parameters->max_effector_position(i)) _position_target(i) = parameters->max_effector_position(i);
        else if (_position_target(i) < parameters->min_effector_position(i)) _position_target(i) = parameters->min_effector_position(i);
        if (_velocity_target(i) > parameters->max_effector_velocity(i)) _velocity_target(i) = parameters->min_effector_velocity(i);
        else if (_velocity_target(i) < parameters->min_effector_velocity(i)) _velocity_target(i) = parameters->min_effector_velocity(i);

        // real state
        if (franka_state->get_effector_position()(i) > parameters->max_effector_position(i))
        {
            if (_velocity_target(i) > 0.0) _velocity_target(i) = 0.0;
            if (_acceleration_target(i) > 0.0) _acceleration_target(i) = 0.0;
        }
        else if (franka_state->get_effector_position()(i) < parameters->min_effector_position(i))
        {
            if (_velocity_target(i) < 0.0) _velocity_target(i) = 0.0;
            if (_acceleration_target(i) < 0.0) _acceleration_target (i) = 0.0;
        }
    }
}

void franka_pole::AccelerationController::_init_step()
{
    _acceleration_target.segment<3>(0) = _input_acceleration;
    _acceleration_target.segment<3>(3) = Eigen::Matrix<double, 3, 1>::Zero();
    _joint_accelerations_target = Eigen::Matrix<double, 7, 1>::Zero();
    _torque = Eigen::Matrix<double, 7, 1>::Zero();
}

void franka_pole::AccelerationController::_compute_jacobians()
{
    // Jacobian
    _jacobian = franka_model->get_effector_jacobian(franka_state->get_joint_positions());
    
    // Jacobian transpose
    _jacobian.transpose();
    
    // Jacobian inverse
    _jacobian_inverse = pseudo_inverse(_jacobian, parameters->target_joint_weights, 0.2);
    
    // Jacobian transpose inverse
    _jacobian_transpose_inverse = pseudo_inverse(_jacobian_transpose, Eigen::Matrix<double, 6, 1>(Eigen::Matrix<double, 6, 1>::Ones()), 0.2);
}

void franka_pole::AccelerationController::_cartesian_control()
{
    // compute cartesian stiffness/damping
    Eigen::Matrix<double, 6, 1> cartesian_stiffness;
    Eigen::Matrix<double, 6, 1> cartesian_damping;
    bool outbound = false;
    for (size_t i = 0; i < 3; i++)
    {
        if (franka_state->get_effector_position()(i) < parameters->min_effector_position(i) || franka_state->get_effector_position()(i) > parameters->max_effector_position(i))
        {
            outbound = true;
            cartesian_stiffness(i) = parameters->outbound_translation_stiffness(i);
            cartesian_damping(i) = parameters->outbound_translation_damping(i);
        }
        else
        {
            cartesian_stiffness(i) = parameters->translation_stiffness(i);
            cartesian_damping(i) = parameters->translation_damping(i);
        }
    }
    cartesian_stiffness.segment<3>(3) = outbound ? parameters->outbound_rotation_stiffness : parameters->rotation_stiffness;
    cartesian_damping.segment<3>(3) = outbound ? parameters->outbound_rotation_damping : parameters->rotation_damping;

    // control
    if (!cartesian_stiffness.isZero() || !cartesian_damping.isZero())
    {
        Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
        error.segment<3>(0) = franka_state->get_effector_position() - _position_target;
        Eigen::Quaterniond orientation = franka_state->get_effector_orientation();
        if (parameters->target_effector_orientation.coeffs().dot(orientation.coeffs()) < 0.0) orientation.coeffs() << -orientation.coeffs();
        Eigen::Quaterniond error_quaternion(orientation.inverse() * parameters->target_effector_orientation);
        error.segment<3>(3) = -(franka_state->get_effector_orientation() * Eigen::Matrix<double, 3, 1>(error_quaternion.x(), error_quaternion.y(), error_quaternion.z()));
        
        Eigen::Matrix<double, 6, 1> control = (-cartesian_stiffness.array() * error.array() - cartesian_damping.array() * (franka_state->get_effector_velocity() - _velocity_target).array()).matrix();
        if (parameters->pure_dynamics) _acceleration_target += control;
        else _torque += Eigen::DiagonalMatrix<double, 7>(parameters->target_joint_weights).inverse() * _jacobian_transpose * control;
    }
}

void franka_pole::AccelerationController::_joint_space_control()
{
    if (!parameters->joint_stiffness.isZero() || !parameters->joint_damping.isZero())
    {
        try
        {
            _joint_positions_target = franka_model->effector_inverse_kinematics(_position_target, parameters->target_effector_orientation, parameters->target_joint_weights, _initial_joint_positions_target);
            _joint_velocities_target = _jacobian_inverse * _velocity_target;
            
            Eigen::Matrix<double, 7, 1> control = (
                parameters->joint_stiffness.array() * (_joint_positions_target - franka_state->get_joint_positions()).array() +
                parameters->joint_damping.array() * (_joint_velocities_target - franka_state->get_joint_velocities()).array()
            ).matrix();
            if (parameters->pure_dynamics) _joint_accelerations_target += control;
            else _torque += control;
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
        }
    }
}

void franka_pole::AccelerationController::_nullspace_control()
{
    if (!parameters->nullspace_stiffness.isZero() || !parameters->nullspace_damping.isZero())
    {
        _torque += (Eigen::Matrix<double, 7, 7>::Identity() - _jacobian_transpose * _jacobian_transpose_inverse) * (
            parameters->nullspace_stiffness.array() * (_initial_joint_positions_target - franka_state->get_joint_positions()).array() +
            parameters->nullspace_damping.array() * (/*zero velocity*/ - franka_state->get_joint_velocities()).array()
        ).matrix();
    }
}

void franka_pole::AccelerationController::_inverse_dynamics_control()
{
    //Add centroidal acceleration to cartesian acceleration, add cartesian acceleration to joint acceleration
    Eigen::Matrix<double, 3, 1> centroidal = franka_model->get_effector_centroidal_acceleration(franka_state->get_joint_positions(), franka_state->get_joint_velocities());
    _acceleration_target.segment<3>(0) += centroidal;
    _joint_accelerations_target += _jacobian_inverse * _acceleration_target;
   
    if (get_model_freedom(parameters->model) == 0)
    {
        // calculate gravity and coriolis
        Eigen::Matrix<double, 9, 1> gravity = franka_model->get_gravity9(franka_state->get_joint_positions());
        Eigen::Matrix<double, 9, 1> coriolis = franka_model->get_coriolis9(franka_state->get_joint_positions(), franka_state->get_joint_velocities());
        _torque += (gravity.segment<7>(0) + coriolis.segment<7>(0));

        if (parameters->dynamics > 0.0)
        {
            // calculate all (7+2) of acceleration
            Eigen::Matrix<double, 9, 1> a9;
            a9.segment<7>(0) = _joint_accelerations_target;
            a9.segment<2>(7) = Eigen::Matrix<double, 2, 1>::Zero();

            // calculate mass matrix
            Eigen::Matrix<double, 9, 9> mass = franka_model->get_mass_matrix9(franka_state->get_joint_positions());

            // calculate first (7) torque
            _torque += parameters->dynamics * (mass.block<7,9>(0,0) * a9);
        }
    }
    else if (get_model_freedom(parameters->model) == 1)
    {
        // calculate gravity and coriolis
        Eigen::Matrix<double, 10, 1> gravity = franka_model->get_gravity10(franka_state->get_joint_positions(), pole_state->get_joint_angle());
        Eigen::Matrix<double, 10, 1> coriolis = franka_model->get_coriolis10(franka_state->get_joint_positions(), franka_state->get_joint_velocities(), pole_state->get_joint_angle(), pole_state->get_joint_dangle());
        _torque += (gravity.segment<7>(0) + coriolis.segment<7>(0));

        if (parameters->dynamics > 0.0)
        {
            // calculate first (7+2) of acceleration
            Eigen::Matrix<double, 10, 1> a10;
            a10.segment<7>(0) = _joint_accelerations_target;
            a10.segment<2>(7) = Eigen::Matrix<double, 2, 1>::Zero();

            // calculate mass matrix
            Eigen::Matrix<double, 10, 10> mass = franka_model->get_mass_matrix10(franka_state->get_joint_positions(), pole_state->get_joint_angle());

            // calculate last (1) of acceleration
            a10.segment<1>(9) = /*zero torque on pole*/ - gravity.segment<1>(9) - coriolis.segment<1>(9) - mass.block<1, 9>(9,0) * a10.segment<9>(0);

            // calculate first (7) torque
            _torque += parameters->dynamics * (mass.block<7,10>(0,0) * a10);
        }
    }
    else
    {
        // calculate gravity and coriolis
        Eigen::Matrix<double, 11, 1> gravity = franka_model->get_gravity11(franka_state->get_joint_positions(), pole_state->get_joint_angle());
        Eigen::Matrix<double, 11, 1> coriolis = franka_model->get_coriolis11(franka_state->get_joint_positions(), franka_state->get_joint_velocities(), pole_state->get_joint_angle(), pole_state->get_joint_dangle());
        _torque += (gravity.segment<7>(0) + coriolis.segment<7>(0));

        if (parameters->dynamics > 0.0)
        {
            // calculate first (7+2) of acceleration
            Eigen::Matrix<double, 11, 1> a11;
            a11.segment<7>(0) = _joint_accelerations_target;
            a11.segment<2>(7) = Eigen::Matrix<double, 2, 1>::Zero();

            // calculate mass matrix
            Eigen::Matrix<double, 11, 11> mass = franka_model->get_mass_matrix11(franka_state->get_joint_positions(), pole_state->get_joint_angle());

            // calculate last (2) of acceleration
            a11.segment<2>(9) = /*zero torque on pole*/ - gravity.segment<2>(9) - coriolis.segment<2>(9) - mass.block<2, 9>(9,0) * a11.segment<9>(0);

            // calculate first (7) torque
            _torque += parameters->dynamics * (mass.block<7,11>(0,0) * a11);
        }
    }

    //Subtract centroidal acceleration to llok nicer at plots (so acceleration = input [ + cartesian control ])
    _acceleration_target.segment<3>(0) -= centroidal;
}

bool franka_pole::AccelerationController::_init_level1(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    _controller_period_counter = 0;
    _input_acceleration = Eigen::Matrix<double, 3, 1>::Zero();
    _velocity_target = Eigen::Matrix<double, 6, 1>::Zero();
    _position_target = parameters->initial_effector_position;
    const double hint[] = { 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4 };
    _initial_joint_positions_target = franka_model->effector_inverse_kinematics(parameters->initial_effector_position, parameters->initial_effector_orientation, parameters->initial_joint_weights, Eigen::Matrix<double, 7, 1>::Map(hint));
    return _init_level2(robot_hw, node_handle);
}

#ifdef FRANKA_POLE_VELOCITY_INTERFACE
Eigen::Matrix<double, 6, 1> franka_pole::AccelerationController::_get_velocity_level1(const ros::Time &time, const ros::Duration &period)
{
    // compute target
    _controller_period_counter += parameters->command_period;
    if (_controller_period_counter >= parameters->controller_period)
    {
        _acceleration_target = _get_acceleration_level2(time, ros::Duration(0,1000000*parameters->controller_period));
        _controller_period_counter = 0;
    }
    Eigen::Matrix<double, 7, 1> torque = Eigen::Matrix<double, 7, 1>::Zero();

    // integrate
    _velocity_target += period.toSec() * _acceleration_target;
    _position_target += period.toSec() * _velocity_target;

    // apply constraints and safety
    for (size_t i = 0; i < 3; i++)
    {
        if (_velocity_target(i) > parameters->max_effector_velocity(i)) _velocity_target(i) = parameters->min_effector_velocity(i);
        else if (_velocity_target(i) < parameters->min_effector_velocity(i)) _velocity_target(i) = parameters->min_effector_velocity(i);
    }

    // publish
    Eigen::Matrix<double, 6, 1> acceleration_command = Eigen::Matrix<double, 6, 1>::Zero();
    acceleration_command.segment<3>(0) = _acceleration_target;
    Eigen::Matrix<double, 6, 1> velocity_command = Eigen::Matrix<double, 6, 1>::Zero();
    velocity_command.segment<3>(0) = _velocity_target;
    publisher->set_command(
        time,
        _position_target,
        parameters->target_effector_orientation,
        velocity_command,
        acceleration_command,
        Eigen::Matrix<double, 7, 1>::Zero(),
        Eigen::Matrix<double, 7, 1>::Zero(),
        Eigen::Matrix<double, 7, 1>::Zero(),
        Eigen::Matrix<double, 7, 1>::Zero());

    return velocity_command;
}
#else
Eigen::Matrix<double, 7, 1> franka_pole::AccelerationController::_get_torque_level1(const ros::Time &time, const ros::Duration &period)
{
    // preparations
    _update_cartesian_targets(time, period);
    _init_step();
    _compute_jacobians();

    // applying controllers subsequently
    _cartesian_control();
    _joint_space_control();
    _nullspace_control();
    _inverse_dynamics_control();

    // publish
    publisher->set_command(
        time,
        _position_target,
        parameters->target_effector_orientation,
        _velocity_target,
        _acceleration_target,
        _joint_positions_target,
        _joint_velocities_target,
        _joint_accelerations_target,
        _torque);

    return _torque;
}
#endif