#include <franka_pole/external_acceleration_controller.h>
#include <pluginlib/class_list_macros.h>

void franka_pole::ExternalAccelerationController::_command_callback(const franka_pole::CommandAcceleration::ConstPtr &msg)
{
    _acceleration_target = Eigen::Matrix<double, 3, 1>::Map(&msg->command_effector_acceleration[0]);
}

bool franka_pole::ExternalAccelerationController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    if (!AccelerationController::_controller_init(robot_hw, node_handle)) return false;

    _command_subscriber = node_handle.subscribe("/franka_pole/command_acceleration", 10, &ExternalAccelerationController::_command_callback, this);

    return true;
}

void franka_pole::ExternalAccelerationController::starting(const ros::Time &time)
{
    AccelerationController::_controller_starting(time);
}

void franka_pole::ExternalAccelerationController::update(const ros::Time &time, const ros::Duration &period)
{
    AccelerationController::_controller_pre_update(time, period);
    AccelerationController::_controller_post_update(time, period, _acceleration_target);
}

PLUGINLIB_EXPORT_CLASS(franka_pole::ExternalAccelerationController, controller_interface::ControllerBase)