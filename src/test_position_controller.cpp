#include <pinocchio/fwd.hpp>
#include <franka_pole/test_position_controller.h>
#include <franka_pole/parameters.h>
#include <pluginlib/class_list_macros.h>

bool franka_pole::TestPositionController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    if (!PositionController::_controller_init(robot_hw, node_handle)) return false;

    Parameters parameters(node_handle);
    _two_dimensional = parameters.two_dimensional();
    _target_position = parameters.target_effector_position();

    return true;
}

void franka_pole::TestPositionController::starting(const ros::Time &time)
{
    PositionController::_controller_starting(time);
}

void franka_pole::TestPositionController::update(const ros::Time &time, const ros::Duration &period)
{
    PositionController::_controller_pre_update(time, period);

    Eigen::Matrix<double, 3, 1> position_target = _target_position;
    if (_two_dimensional) position_target(0) += 0.25 * sin(2 * M_PI * time.toSec());
    else position_target(1) += 0.5 * sin(2 * M_PI * time.toSec());

    PositionController::_controller_post_update(time, period, position_target, Eigen::Matrix<double, 3, 1>::Zero());
}

PLUGINLIB_EXPORT_CLASS(franka_pole::TestPositionController, controller_interface::ControllerBase)