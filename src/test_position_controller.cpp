#include <franka_pole/test_position_controller.h>
#include <pluginlib/class_list_macros.h>

bool franka_pole::TestPositionController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    if (!PositionController::_controller_init(robot_hw, node_handle)) return false;

    return true;
}

void franka_pole::TestPositionController::starting(const ros::Time &time)
{
    PositionController::_controller_starting(time);
}

void franka_pole::TestPositionController::update(const ros::Time &time, const ros::Duration &period)
{
    PositionController::_controller_pre_update(time, period);
    
    Eigen::Matrix<double, 3, 1> position_target(0.5, 0.5 * sin(2 * M_PI * time.toSec()), 0.5);

    PositionController::_controller_post_update(time, period, position_target, Eigen::Matrix<double, 3, 1>::Zero());
}

PLUGINLIB_EXPORT_CLASS(franka_pole::TestPositionController, controller_interface::ControllerBase)