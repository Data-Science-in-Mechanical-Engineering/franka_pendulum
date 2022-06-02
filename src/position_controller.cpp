#include <franka_pole/position_controller.h>
#include <franka_pole/controller.h>
#include <franka_pole/franka_state.h>
#include <franka_pole/pole_state.h>
#include <franka_pole/publisher.h>
#include <franka_pole/pseudo_inverse.h>
#include <pluginlib/class_list_macros.h>

void franka_pole::PositionController::_command_callback(const franka_pole::CommandPosition::ConstPtr &msg)
{
    _position_target = Eigen::Matrix<double, 3, 1>(msg->franka_effector_x, msg->franka_effector_y, msg->franka_effector_z);
    _velocity_target = Eigen::Matrix<double, 3, 1>(msg->franka_effector_dx, msg->franka_effector_dy, msg->franka_effector_dz);
}

bool franka_pole::PositionController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    if (!_controller_init(robot_hw, node_handle)) return false;

    _cartesian_stiffness.setZero();
    _cartesian_stiffness.diagonal().segment<3>(0) = Eigen::Vector3d::Ones() * 200.0;
    _cartesian_stiffness.diagonal().segment<3>(3) = Eigen::Vector3d::Ones() * 10.0;
    _nullspace_stiffness = 0.5;
    _nullspace_damping = 2.0 * sqrt(_nullspace_stiffness);
    _cartesian_damping = 2.0 * _cartesian_stiffness.array().sqrt().matrix();

    _command_subscriber = node_handle.subscribe("/franka_pole/position_command", 10, &PositionController::_command_callback, this);

    return true;
}

void franka_pole::PositionController::starting(const ros::Time &time)
{
    _controller_starting(time);
}

void franka_pole::PositionController::update(const ros::Time &time, const ros::Duration &period)
{
    _controller_pre_update(time, period);

    //compute target
    Eigen::Quaterniond orientation_target(0.0, 1.0, 0.0, 0.0);
    Eigen::Matrix<double, 6, 1> velocity_target = Eigen::Matrix<double, 6, 1>::Zero();
    velocity_target.segment<3>(0) = _velocity_target;

    // compute error
    Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
    error.segment<3>(0) = franka_state->get_effector_position() - _position_target;
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
    torque += jacobian_transpose * (-_cartesian_stiffness * error - _cartesian_damping * (franka_state->get_effector_velocity() - velocity_target));
    torque += (Eigen::Matrix<double, 7, 7>::Identity() - jacobian_transpose * pseudo_inverse(jacobian_transpose, true)) *
      (_nullspace_stiffness * (q_target - franka_state->get_joint_positions()) - _nullspace_damping * franka_state->get_joint_velocities());

    // set control
    franka_state->set_torque(torque);

    // publish
    publisher->set_control_timestamp(time);
    publisher->set_control_effector_position(_position_target);
    publisher->set_control_effector_velocity(_velocity_target);
    publisher->set_control_effector_acceleration(Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0));

    _controller_post_update(time, period);
}

PLUGINLIB_EXPORT_CLASS(franka_pole::PositionController, controller_interface::ControllerBase)