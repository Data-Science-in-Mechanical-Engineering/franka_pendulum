#include <franka_pole/test_acceleration_controller.h>
#include <franka_pole/parameters.h>

bool franka_pole::TestAccelerationController::_init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{    
    return true;
}

Eigen::Matrix<double, 3, 1> franka_pole::TestAccelerationController::_get_acceleration_level2(const ros::Time &time, const ros::Duration &period)
{
    Eigen::Matrix<double, 3, 1> acceleration_target = Eigen::Matrix<double, 3, 1>::Zero();
    unsigned int axis = (parameters->model == Model::D2 || parameters->model == Model::D2b) ? 0 : 1;
    acceleration_target(axis) = 0.1 * (2*M_PI) * (2*M_PI) * (cos(2 * M_PI * time.toSec()) > 0.0 ? 1.0 : -1.0);
    return acceleration_target;
}

FRANKA_POLE_CONTROLLER_IMPLEMENTATION(franka_pole::TestAccelerationController);