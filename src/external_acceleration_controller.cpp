#include <franka_pendulum/external_acceleration_controller.h>
#include <franka_pendulum/parameters.h>

void franka_pendulum::ExternalAccelerationController::_callback(const franka_pendulum::CommandAcceleration::ConstPtr &msg)
{
    std::lock_guard<std::mutex> guard(mutex);
    _acceleration_target = Eigen::Matrix<double, 3, 1>::Map(&msg->command_effector_acceleration[0]);
}

bool franka_pendulum::ExternalAccelerationController::_init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    _acceleration_target = Eigen::Matrix<double, 3, 1>::Zero();
    if (!_subscribed)
    {
        _subscriber = node_handle.subscribe("/" + parameters->namespacee + "/command_acceleration", 10, &ExternalAccelerationController::_callback, this);
        _subscribed = true;
    }
    return true;
}

Eigen::Matrix<double, 3, 1> franka_pendulum::ExternalAccelerationController::_get_acceleration_level2(const ros::Time &time, const ros::Duration &period)
{
    return _acceleration_target;
}

FRANKA_POLE_CONTROLLER_IMPLEMENTATION(franka_pendulum::ExternalAccelerationController);