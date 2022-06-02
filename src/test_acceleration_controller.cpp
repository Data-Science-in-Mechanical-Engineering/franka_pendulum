#include <franka_pole/test_acceleration_controller.h>
#include <pluginlib/class_list_macros.h>

bool franka_pole::TestAccelerationController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    if (!AccelerationController::_controller_init(robot_hw, node_handle)) return false;
    
    return true;
}

void franka_pole::TestAccelerationController::starting(const ros::Time &time)
{
    AccelerationController::_controller_starting(time);
}

void franka_pole::TestAccelerationController::update(const ros::Time &time, const ros::Duration &period)
{
    AccelerationController::_controller_pre_update(time, period);
    Eigen::Matrix<double, 3, 1> acceleration_target(0.0, 0.5 * (2*M_PI) * (2*M_PI) * cos(2 * M_PI * time.toSec()), 0.0);
    AccelerationController::_controller_post_update(time, period, acceleration_target);
}

PLUGINLIB_EXPORT_CLASS(franka_pole::TestAccelerationController, controller_interface::ControllerBase)