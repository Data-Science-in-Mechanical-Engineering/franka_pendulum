#include <franka_pole/acceleration_controller.h>
#include <franka_pole/franka_state.h>
#include <franka_pole/pole_state.h>
#include <franka_pole/publisher.h>
#include <franka_pole/pseudo_inverse.h>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <ros/package.h>

bool franka_pole::AccelerationController::_controller_init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    if (!Controller::_controller_init(robot_hw, node_handle)) return false;

    _cartesian_stiffness.setZero();
    _cartesian_stiffness.diagonal().segment<3>(0) = get_translation_stiffness();
    _cartesian_stiffness.diagonal().segment<3>(3) = get_rotation_stiffness();
    _cartesian_damping = 2.0 * _cartesian_stiffness.array().sqrt().matrix();

    std::string package_path = ros::package::getPath("franka_pole");
    try { pinocchio::urdf::buildModel(package_path + (is_two_dimensional() ? "/robots/franka_pole_2D.urdf" : "/robots/franka_pole.urdf"), _pinocchio_model); }
    catch (const std::exception &ex) { ROS_ERROR_STREAM("CartesianController: Exception building pinocchio model: " << ex.what()); return false; }
    _pinocchio_data = pinocchio::Data(_pinocchio_model);
    for (size_t i = 0; i < 7; i++)
    {
        std::string name = get_arm_id() + "_joint" + std::to_string(i + 1);
        if (!_pinocchio_model.existJointName(name)) { ROS_ERROR_STREAM("CartesianController: Joint " + get_arm_id() + "_joint" + std::to_string(i + 1) + " not found"); return false; }
        _pinocchio_joint_ids[i] = _pinocchio_model.getJointId(name);
    }
    for (size_t i = 0; i < 2; i++)
    {
        std::string name = get_arm_id() + "_finger_joint" + std::to_string(i + 1);
        if (!_pinocchio_model.existJointName(name)) { ROS_ERROR_STREAM("CartesianController: Joint " + get_arm_id() + "_finger_joint" + std::to_string(i + 1) + " not found"); return false; }
        _pinocchio_joint_ids[i+7] = _pinocchio_model.getJointId(name);
    }
    if (is_two_dimensional())
    {
        if (!_pinocchio_model.existJointName(get_arm_id() + "_pole_joint_y")) { ROS_ERROR_STREAM("CartesianController: Joint " + get_arm_id() + "_pole_joint_y not found"); return false; }
        _pinocchio_joint_ids[9] = _pinocchio_model.getJointId(get_arm_id() + "_pole_joint_y");
        if (!_pinocchio_model.existJointName(get_arm_id() + "_pole_joint_x")) { ROS_ERROR_STREAM("CartesianController: Joint " + get_arm_id() + "_pole_joint_x not found"); return false; }
        _pinocchio_joint_ids[10] = _pinocchio_model.getJointId(get_arm_id() + "_pole_joint_x");
    }
    else
    {
        if (!_pinocchio_model.existJointName(get_arm_id() + "_pole_joint_x")) { ROS_ERROR_STREAM("CartesianController: Joint " + get_arm_id() + "_pole_joint_x not found"); return false; }
        _pinocchio_joint_ids[9] = _pinocchio_model.getJointId(get_arm_id() + "_pole_joint_x");
    }
    _pinocchio_model.gravity = pinocchio::Motion::Zero();
    
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
    Eigen::Vector3d position_target = get_box_center();
    Eigen::Quaterniond orientation_target(0.0, 1.0, 0.0, 0.0);
    if (std::max(get_box_min()(1), std::min(franka_state->get_effector_position()(1), get_box_max()(1))) != franka_state->get_effector_position()(1))
    {
        position_target(1) = std::max(get_box_min()(1), std::min(franka_state->get_effector_position()(1), get_box_max()(1))); //We care about Y
        _cartesian_stiffness.diagonal()(1) = get_translation_stiffness()(1);
        _cartesian_damping.diagonal()(1) = 2 * sqrt(get_translation_stiffness()(1));
    }
    else
    {
        _cartesian_stiffness.diagonal()(1) = 0.0; //We don't care about Y
        _cartesian_damping.diagonal()(1) = 0.0;
    }
    if (is_two_dimensional())
    {
        if (std::max(get_box_min()(0), std::min(franka_state->get_effector_position()(0), get_box_max()(0))) != franka_state->get_effector_position()(0))
        {
            position_target(0) = std::max(get_box_min()(0), std::min(franka_state->get_effector_position()(0), get_box_max()(0))); //We care about X
            _cartesian_stiffness.diagonal()(0) = get_translation_stiffness()(0);
            _cartesian_damping.diagonal()(0) = 2 * sqrt(get_translation_stiffness()(0));
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
    (get_nullspace_stiffness().array() * (get_initial_joint_positions().segment<7>(0) - franka_state->get_joint_positions()).array() - 2 * get_nullspace_stiffness().array().sqrt() * franka_state->get_joint_velocities().array()).matrix();

    // acceleration control
    if (is_two_dimensional())
    {
        Eigen::Matrix<double, 11, 1> q11 = Eigen::Matrix<double, 11, 1>::Zero();
        q11.segment<7>(0) = franka_state->get_joint_positions();
        q11(9) = pole_state->get_angle()(1);
        q11(10) = pole_state->get_angle()(0);
        Eigen::Matrix<double, 11, 1> v11 = Eigen::Matrix<double, 11, 1>::Zero();
        v11.segment<7>(0) = franka_state->get_joint_velocities();
        v11(9) = pole_state->get_dangle()(1);
        v11(10) = pole_state->get_dangle()(0);
        Eigen::Matrix<double, 6, 1> a6 = Eigen::Matrix<double, 6, 1>::Zero();
        a6.segment<3>(0) = acceleration_target;
        Eigen::Matrix<double, 11, 1> a11 = Eigen::Matrix<double, 11, 1>::Zero();
        a11.segment<7>(0) = pseudo_inverse(jacobian, true) * a6;
        //a11(9) = ???
        //a11(10) = ???
        pinocchio::rnea(_pinocchio_model, _pinocchio_data, q11, v11, a11);
        torque += _pinocchio_data.tau.segment<7>(0);
    }
    else
    {
        Eigen::Matrix<double, 10, 1> q10 = Eigen::Matrix<double, 10, 1>::Zero();
        q10.segment<7>(0) = franka_state->get_joint_positions();
        q10(9) = pole_state->get_angle()(0);
        Eigen::Matrix<double, 10, 1> v10 = Eigen::Matrix<double, 10, 1>::Zero();
        v10.segment<7>(0) = franka_state->get_joint_velocities();
        v10(9) = pole_state->get_dangle()(0);
        Eigen::Matrix<double, 6, 1> a6 = Eigen::Matrix<double, 6, 1>::Zero();
        a6.segment<3>(0) = acceleration_target;
        Eigen::Matrix<double, 10, 1> a10 = Eigen::Matrix<double, 10, 1>::Zero();
        a10.segment<7>(0) = pseudo_inverse(jacobian, true) * a6;
        //a10(9) = ???
        pinocchio::rnea(_pinocchio_model, _pinocchio_data, q10, v10, a10);
        torque += _pinocchio_data.tau.segment<7>(0);
    }

    // publish
    publisher->set_command_timestamp(time);
    publisher->set_command_effector_position(position_target);
    publisher->set_command_effector_velocity(Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0));
    publisher->set_command_effector_acceleration(acceleration_target);

    Controller::_controller_post_update(time, period, torque);
}