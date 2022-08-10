#include <franka_pole/test_position_controller.h>
#include <franka_pole/parameters.h>

bool franka_pole::TestPositionController::_init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{    
    return true;
}

Eigen::Matrix<double, 3, 1> franka_pole::TestPositionController::_get_position_level2(const ros::Time &time, const ros::Duration &period)
{
    Eigen::Matrix<double, 3, 1> position_target = parameters->target_effector_position;
    unsigned int axis = (parameters->model == Model::D2 || parameters->model == Model::D2b) ? 0 : 1;
    position_target(axis) += 0.1 * sin(2 * M_PI * time.toSec());
    return position_target;
}

Eigen::Matrix<double, 3, 1> franka_pole::TestPositionController::_get_velocity_level2(const ros::Time &time, const ros::Duration &period)
{
    return Eigen::Matrix<double, 3, 1>::Zero();
}

FRANKA_POLE_CONTROLLER_IMPLEMENTATION(franka_pole::TestPositionController);