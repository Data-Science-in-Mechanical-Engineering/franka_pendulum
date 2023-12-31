#include <franka_pendulum/external_position_controller.h>
#include <franka_pendulum/parameters.h>

void franka_pendulum::ExternalPositionController::_callback(const franka_pendulum::CommandPosition::ConstPtr &msg)
{
    std::lock_guard<std::mutex> guard(mutex);
    _position_target = Eigen::Matrix<double, 3, 1>::Map(&msg->command_effector_position[0]);
    _velocity_target = Eigen::Matrix<double, 3, 1>::Map(&msg->command_effector_velocity[0]);
}

bool franka_pendulum::ExternalPositionController::_init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    _position_target = parameters->target_effector_position;
    _velocity_target = Eigen::Matrix<double, 3, 1>::Zero();
    if (!_subscribed)
    {
        _subscriber = node_handle.subscribe("/" + parameters->namespacee + "/command_position", 10, &ExternalPositionController::_callback, this);
        _subscribed = true;
    }
    return true;
}

Eigen::Matrix<double, 3, 1> franka_pendulum::ExternalPositionController::_get_position_level2(const ros::Time &time, const ros::Duration &period)
{
    return _position_target;
}

Eigen::Matrix<double, 3, 1> franka_pendulum::ExternalPositionController::_get_velocity_level2(const ros::Time &time, const ros::Duration &period)
{
    return _velocity_target;
}

FRANKA_POLE_CONTROLLER_IMPLEMENTATION(franka_pendulum::ExternalPositionController);