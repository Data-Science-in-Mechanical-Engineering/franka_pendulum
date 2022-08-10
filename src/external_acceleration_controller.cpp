#include <franka_pole/external_acceleration_controller.h>
#include <franka_pole/parameters.h>

void franka_pole::ExternalAccelerationController::_callback(const franka_pole::CommandAcceleration::ConstPtr &msg)
{
    std::lock_guard<std::mutex> guard(mutex);
    _acceleration_target = Eigen::Matrix<double, 3, 1>::Map(&msg->command_effector_acceleration[0]);
}

bool franka_pole::ExternalAccelerationController::_init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    _acceleration_target = Eigen::Matrix<double, 3, 1>::Zero();
    _subscriber = node_handle.subscribe("/" + parameters->namespacee + "/command_acceleration", 10, &ExternalAccelerationController::_callback, this);
    return true;
}

Eigen::Matrix<double, 3, 1> franka_pole::ExternalAccelerationController::_get_acceleration_level2(const ros::Time &time, const ros::Duration &period)
{
    return _acceleration_target;
}

FRANKA_POLE_CONTROLLER_IMPLEMENTATION(franka_pole::ExternalAccelerationController);