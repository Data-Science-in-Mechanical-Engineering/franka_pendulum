#include <pinocchio/fwd.hpp>
#include <franka_pole/test_position_controller.h>
#include <franka_pole/parameters.h>
#include <pluginlib/class_list_macros.h>

bool franka_pole::TestPositionController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    if (!PositionController::_controller_init(robot_hw, node_handle)) return false;

    Parameters parameters(node_handle);
    _model = parameters.model();
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
    if (_model == Model::D2 || _model == Model::D2b) position_target(0) += 0.25 * (cos(2 * M_PI * time.toSec()) > 0.0 ? 1.0 : -1.0);
    else position_target(1) += 0.25 * (cos(2 * M_PI * time.toSec()) > 0.0 ? 1.0 : -1.0);

    PositionController::_controller_post_update(time, period, position_target, Eigen::Matrix<double, 3, 1>::Zero());
}

PLUGINLIB_EXPORT_CLASS(franka_pole::TestPositionController, controller_interface::ControllerBase)