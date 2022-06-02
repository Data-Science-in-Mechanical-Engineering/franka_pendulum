#include <franka_pole/external_position_controller.h>
#include <pluginlib/class_list_macros.h>

void franka_pole::ExternalPositionController::_command_callback(const franka_pole::CommandPosition::ConstPtr &msg)
{
    _position_target = Eigen::Matrix<double, 3, 1>(msg->franka_effector_x, msg->franka_effector_y, msg->franka_effector_z);
    _velocity_target = Eigen::Matrix<double, 3, 1>(msg->franka_effector_dx, msg->franka_effector_dy, msg->franka_effector_dz);
}

bool franka_pole::ExternalPositionController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    if (!PositionController::_controller_init(robot_hw, node_handle)) return false;

    _command_subscriber = node_handle.subscribe("/franka_pole/command_position", 10, &ExternalPositionController::_command_callback, this);

    return true;
}

void franka_pole::ExternalPositionController::starting(const ros::Time &time)
{
    PositionController::_controller_starting(time);
}

void franka_pole::ExternalPositionController::update(const ros::Time &time, const ros::Duration &period)
{
    PositionController::_controller_pre_update(time, period);
    PositionController::_controller_post_update(time, period, _position_target, _velocity_target);
}

PLUGINLIB_EXPORT_CLASS(franka_pole::ExternalPositionController, controller_interface::ControllerBase)