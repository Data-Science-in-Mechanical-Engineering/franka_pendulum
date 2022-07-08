#include <franka_pole/test_acceleration_controller.h>
#include <franka_pole/parameters.h>
#include <pluginlib/class_list_macros.h>

bool franka_pole::TestAccelerationController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    if (!AccelerationController::_controller_init(robot_hw, node_handle)) return false;
    
    Parameters parameters(node_handle);
    _two_dimensional = parameters.two_dimensional();
    
    return true;
}

void franka_pole::TestAccelerationController::starting(const ros::Time &time)
{
    AccelerationController::_controller_starting(time);
}

void franka_pole::TestAccelerationController::update(const ros::Time &time, const ros::Duration &period)
{
    AccelerationController::_controller_pre_update(time, period);

    Eigen::Matrix<double, 3, 1> acceleration_target = Eigen::Matrix<double, 3, 1>::Zero();
    //acceleration_target = Eigen::Matrix<double, 3, 1>(0.0, 10, 0.0);
    if (_two_dimensional) acceleration_target(0) = 0.25 * (2*M_PI) * (2*M_PI) * cos(2 * M_PI * time.toSec());
    else acceleration_target(1) = 0.25 * (2*M_PI) * (2*M_PI) * cos(2 * M_PI * time.toSec());
    
    AccelerationController::_controller_post_update(time, period, acceleration_target);
}

PLUGINLIB_EXPORT_CLASS(franka_pole::TestAccelerationController, controller_interface::ControllerBase)