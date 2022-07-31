#include <franka_pole/external_position_controller.h>
#include <franka_pole/parameters.h>

void franka_pole::ExternalPositionController::_callback(const franka_pole::CommandPosition::ConstPtr &msg)
{
    std::lock_guard<std::mutex> guard(_mutex);
    _position_target = Eigen::Matrix<double, 3, 1>::Map(&msg->command_effector_position[0]);
    _velocity_target = Eigen::Matrix<double, 3, 1>::Map(&msg->command_effector_velocity[0]);
}

bool franka_pole::ExternalPositionController::_init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    std::lock_guard<std::mutex> guard(_mutex);
    _position_target = parameters->target_effector_position;
    _velocity_target = Eigen::Matrix<double, 3, 1>::Zero();
    _subscriber = node_handle.subscribe("/franka_pole/command_position", 10, &ExternalPositionController::_callback, this);
    return true;
}

Eigen::Matrix<double, 3, 1> franka_pole::ExternalPositionController::_get_position_level2(const ros::Time &time, const ros::Duration &period)
{
    std::lock_guard<std::mutex> guard(_mutex);
    return _position_target;
}

Eigen::Matrix<double, 3, 1> franka_pole::ExternalPositionController::_get_velocity_level2(const ros::Time &time, const ros::Duration &period)
{
    std::lock_guard<std::mutex> guard(_mutex);
    return _velocity_target;
}

FRANKA_POLE_CONTROLLER_IMPLEMENTATION(franka_pole::ExternalPositionController);