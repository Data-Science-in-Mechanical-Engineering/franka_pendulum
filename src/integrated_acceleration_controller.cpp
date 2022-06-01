#include <franka_pole/integrated_acceleration_controller.h>
#include <franka_pole/controller.h>
#include <franka_pole/franka_state.h>
#include <franka_pole/pole_state.h>
#include <franka_pole/publisher.h>
#include <franka_pole/pseudo_inverse.h>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <ros/package.h>
#include <pluginlib/class_list_macros.h>

bool franka_pole::IntegratedAccelerationController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    if (!_controller_init(robot_hw, node_handle)) return false;

    _cartesian_stiffness.setZero();
    _cartesian_stiffness.diagonal().segment<3>(0) = Eigen::Vector3d::Ones() * 200.0;
    _cartesian_stiffness.diagonal().segment<3>(3) = Eigen::Vector3d::Ones() * 10.0;
    _nullspace_stiffness = 0.5;
    _nullspace_damping = 2.0 * sqrt(_nullspace_stiffness);
    _cartesian_damping = 2.0 * _cartesian_stiffness.array().sqrt().matrix();

    std::string package_path = ros::package::getPath("franka_pole");
    try { pinocchio::urdf::buildModel(package_path + "/robots/franka_pole.urdf", _pinocchio_model); }
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
    if (!_pinocchio_model.existJointName(get_arm_id() + "_lower_upper")) { ROS_ERROR_STREAM("CartesianController: Joint " + get_arm_id() + "_lower_upper not found"); return false; }
    _pinocchio_joint_ids[9] = _pinocchio_model.getJointId(get_arm_id() + "_lower_upper");
    _pinocchio_model.gravity = pinocchio::Motion::Zero();

    return true;
}

void franka_pole::IntegratedAccelerationController::starting(const ros::Time &time)
{
    _controller_starting(time);
}

void franka_pole::IntegratedAccelerationController::update(const ros::Time &time, const ros::Duration &period)
{
    _controller_pre_update(time, period);

    // compute target
    Eigen::Vector3d position_target(0.5, 0.0, 0.5);
    Eigen::Quaterniond orientation_target(0.0, 1.0, 0.0, 0.0);
    if (franka_state->get_effector_position()(1) < -0.6 || franka_state->get_effector_position()(1) > 0.6)
    {
        position_target(1) = (franka_state->get_effector_position()(1) > 0.0) ? 0.6 : -0.6; //We care about Y
        _cartesian_stiffness.diagonal()(1) = 200.0;
        _cartesian_damping.diagonal()(1) = 10.0;
    }
    else
    {
        _cartesian_stiffness.diagonal()(1) = 0.0; //We don't care about Y
        _cartesian_damping.diagonal()(1) = 0.0;
    }

    // compute error
    Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
    error.segment<3>(0) = franka_state->get_effector_position() - position_target;
    Eigen::Quaterniond orientation = franka_state->get_effector_orientation();
    if (orientation_target.coeffs().dot(orientation.coeffs()) < 0.0) orientation.coeffs() << -orientation.coeffs();
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_target);
    error.segment<3>(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error.segment<3>(3) = -franka_state->get_effector_transform().linear() * error.segment<3>(3);

    //compute desired q
    double q_target_raw[] = { 0.0, 0.0, 0.0, -M_PI/2, 0.0, M_PI/2, M_PI/4 };
    Eigen::Matrix<double, 7, 1> q_target = Eigen::Matrix<double, 7, 1>::Map(q_target_raw);

    // compute control
    Eigen::Matrix<double, 7, 1> torque = franka_state->get_coriolis();
    Eigen::Matrix<double, 7, 6> jacobian_transpose = franka_state->get_effector_jacobian().transpose();
    torque += jacobian_transpose * (-_cartesian_stiffness * error - _cartesian_damping * franka_state->get_effector_velocity());
    torque += (Eigen::Matrix<double, 7, 7>::Identity() - jacobian_transpose * pseudo_inverse(jacobian_transpose, true)) *
      (_nullspace_stiffness * (q_target - franka_state->get_joint_positions()) - _nullspace_damping * franka_state->get_joint_velocities());

    // acceleration control
    double desired_ddy = -_a * pole_state->get_angle() + -_b * pole_state->get_dangle() + _c * franka_state->get_effector_position()(1) + _d * franka_state->get_effector_velocity()(1);
    Eigen::Matrix<double, 10, 1> q10 = Eigen::Matrix<double, 10, 1>::Zero();
    q10.segment<7>(0) = franka_state->get_joint_positions();
    q10(9) = pole_state->get_angle();
    Eigen::Matrix<double, 10, 1> v10 = Eigen::Matrix<double, 10, 1>::Zero();
    v10.segment<7>(0) = franka_state->get_joint_velocities();
    v10(9) = pole_state->get_dangle();
    Eigen::Matrix<double, 10, 1> a10 = Eigen::Matrix<double, 10, 1>::Zero();
    Eigen::Matrix<double, 6, 1> desired_effector_acceleration = Eigen::Matrix<double, 6, 1>::Zero();
    desired_effector_acceleration(1) = desired_ddy;
    a10.segment<7>(0) = jacobian_transpose * desired_effector_acceleration;
    //a10(9) = ???
    pinocchio::rnea(_pinocchio_model, _pinocchio_data, q10, v10, a10);
    torque += _pinocchio_data.tau.segment<7>(0);

    // set control
    franka_state->set_torque(torque);

    // publish
    publisher->set_control_timestamp(time);
    publisher->set_control_effector_position(position_target);
    publisher->set_control_effector_velocity(Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0));
    publisher->set_control_effector_acceleration(Eigen::Matrix<double, 3, 1>(0.0, desired_ddy, 0.0));

    _controller_post_update(time, period);
}

PLUGINLIB_EXPORT_CLASS(franka_pole::IntegratedAccelerationController, controller_interface::ControllerBase)