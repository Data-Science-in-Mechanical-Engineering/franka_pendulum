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
    _model = parameters.model();
    _cartesian_stiffness.segment<3>(0) = parameters.translation_stiffness();
    _cartesian_stiffness.segment<3>(3) = parameters.rotation_stiffness();
    _cartesian_damping = 2.0 * _cartesian_stiffness.array().sqrt().matrix();
    _cartesian_stiffness_safety = parameters.translation_stiffness_safety();
    _cartesian_damping_safety = 2.0 * _cartesian_stiffness_safety.array().sqrt().matrix();
    _nullspace_stiffness = parameters.nullspace_stiffness();
    _position_target = parameters.initial_effector_position();
    _orientation_target = parameters.target_effector_orientation();
    _max_effector_position = parameters.max_effector_position();
    _min_effector_position = parameters.min_effector_position();
    _target_joint_positions = franka_model->effector_inverse_kinematics(parameters.target_effector_position(), parameters.target_effector_orientation(), parameters.initial_joint0_position());

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
    // compute target
    _velocity_target += 0.001 * acceleration_target;
    _position_target += 0.001 * _velocity_target;
    Eigen::Matrix<double, 6, 1> a6 = Eigen::Matrix<double, 6, 1>::Zero();
    a6.segment<3>(0) = acceleration_target;

    // apply constraints and safety
    Eigen::Matrix<double, 6, 6> cartesian_stiffness = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> cartesian_damping = Eigen::Matrix<double, 6, 6>::Zero();
    cartesian_stiffness.diagonal().segment<3>(3) = _cartesian_stiffness.segment<3>(3);
    cartesian_damping.diagonal().segment<3>(3) = _cartesian_damping.segment<3>(3);
    for (size_t i = 0; i < 3; i++)
    {
        if (_position_target(i) > _max_effector_position(i) || franka_state->get_effector_position()(i) > _max_effector_position(i))
        {
            _position_target(i) = _max_effector_position(i);
            if (_velocity_target(i) > 0.0) _velocity_target(i) = 0.0;
            if (a6(i) > 0.0) a6(i) = 0.0;
        }
        else if (_position_target(i) < _min_effector_position(i) || franka_state->get_effector_position()(i) < _min_effector_position(i))
        {
            _position_target(i) = _min_effector_position(i);
            if (_velocity_target(i) < 0.0) _velocity_target(i) = 0.0;
            if (a6(i) < 0.0) a6(i) = 0.0;
        }

        if (_position_target(i) < _min_effector_position(i) || _position_target(i) > _max_effector_position(i))
        {
            cartesian_stiffness(i,i) = _cartesian_stiffness_safety(i);
            cartesian_damping(i,i) = _cartesian_damping_safety(i);
        }
        else
        {
            cartesian_stiffness(i,i) = _cartesian_stiffness(i);
            cartesian_damping(i,i) = _cartesian_damping(i);
        }
    }
    
    std::cout << cartesian_stiffness << std::endl << std::endl;

    // calculate jacobians
    Eigen::Matrix<double, 6, 7> jacobian = franka_model->get_effector_jacobian(franka_state->get_joint_positions());
    Eigen::Matrix<double, 7, 6> jacobian_transpose = jacobian.transpose();
    Eigen::Matrix<double, 7, 6> jacobian_inverse = pseudo_inverse(jacobian, true);
    Eigen::Matrix<double, 6, 7> jacobian_transpose_inverse = pseudo_inverse(jacobian_transpose, true);

    // conventional control
    Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
    error.segment<3>(0) = franka_state->get_effector_position() - _position_target;
    Eigen::Quaterniond orientation = franka_state->get_effector_orientation();
    if (_orientation_target.coeffs().dot(orientation.coeffs()) < 0.0) orientation.coeffs() << -orientation.coeffs();
    Eigen::Quaterniond error_quaternion(orientation.inverse() * _orientation_target);
    error.segment<3>(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error.segment<3>(3) = -(franka_state->get_effector_orientation() * error.segment<3>(3));
    Eigen::Matrix<double, 6, 1> v6 = Eigen::Matrix<double, 6, 1>::Zero();
    v6.segment<3>(0) = _velocity_target;
    Eigen::Matrix<double, 7, 1> torque = jacobian_transpose * (-cartesian_stiffness * error - cartesian_damping * (franka_state->get_effector_velocity() - v6));

    // nullspace control
    torque += (Eigen::Matrix<double, 7, 7>::Identity() - jacobian_transpose * jacobian_transpose_inverse) *
    (_nullspace_stiffness.array() * (_target_joint_positions - franka_state->get_joint_positions()).array() - 2 * _nullspace_stiffness.array().sqrt() * franka_state->get_joint_velocities().array()).matrix();

    // anti-damping
    torque += franka_state->get_joint_velocities() * 0.003;

    // inverse dynamics
    a6.segment<3>(0) += franka_model->get_effector_centroidal_acceleration(franka_state->get_joint_positions(), franka_state->get_joint_velocities());
    if (_model == Model::D2 || _model == Model::D2b)
    {
        // calculate first (7+2) of acceleration
        Eigen::Matrix<double, 11, 1> a11;
        a11.segment<7>(0) = jacobian_inverse * a6;
        a11.segment<2>(7) = Eigen::Matrix<double, 2, 1>::Zero();

        // calculate gravity, coriolis and mass matrix
        Eigen::Matrix<double, 11, 1> gravity = franka_model->get_gravity11(franka_state->get_joint_positions(), pole_state->get_joint_angle());
        Eigen::Matrix<double, 11, 1> coriolis = franka_model->get_coriolis11(franka_state->get_joint_positions(), franka_state->get_joint_velocities(), pole_state->get_joint_angle(), pole_state->get_joint_dangle());
        Eigen::Matrix<double, 11, 11> mass = franka_model->get_mass_matrix11(franka_state->get_joint_positions(), pole_state->get_joint_angle());

        // calculate last (1) of acceleration
        a11.segment<2>(9) = /*zero torque on pole*/ - gravity.segment<2>(9) - coriolis.segment<2>(9) - mass.block<2, 9>(9,0) * a11.segment<9>(0);

        // calculate first (7) torque (we don't care about 2 on fingers)
        torque += (mass.block<7,11>(0,0) * a11 + coriolis.segment<7>(0) + gravity.segment<7>(0));
    }
    else
    {
        // calculate first (7+2) of acceleration
        Eigen::Matrix<double, 10, 1> a10;
        a10.segment<7>(0) = jacobian_inverse * a6;
        a10.segment<2>(7) = Eigen::Matrix<double, 2, 1>::Zero();

        // calculate gravity, coriolis and mass matrix
        Eigen::Matrix<double, 10, 1> gravity = franka_model->get_gravity10(franka_state->get_joint_positions(), pole_state->get_joint_angle());
        Eigen::Matrix<double, 10, 1> coriolis = franka_model->get_coriolis10(franka_state->get_joint_positions(), franka_state->get_joint_velocities(), pole_state->get_joint_angle(), pole_state->get_joint_dangle());
        Eigen::Matrix<double, 10, 10> mass = franka_model->get_mass_matrix10(franka_state->get_joint_positions(), pole_state->get_joint_angle());

        // calculate last (1) of acceleration
        a10.segment<1>(9) = /*zero torque on pole*/ - gravity.segment<1>(9) - coriolis.segment<1>(9) - mass.block<1, 9>(9,0) * a10.segment<9>(0);

        // calculate first (7) torque (we don't care about 2 on fingers)
        torque += (mass.block<7,10>(0,0) * a10 + coriolis.segment<7>(0) + gravity.segment<7>(0));
    }
    
    // publish
    publisher->set_command_effector_position(_position_target);
    publisher->set_command_effector_velocity(_velocity_target);
    publisher->set_command_effector_acceleration(acceleration_target);

    Controller::_controller_post_update(time, period, torque);
}