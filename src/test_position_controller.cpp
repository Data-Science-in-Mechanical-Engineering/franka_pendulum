#include <franka_pole/test_position_controller.h>
#include <franka_pole/parameters.h>

bool franka_pole::TestPositionController::_init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    _time = ros::Time(0,0);
    return true;
}

Eigen::Matrix<double, 3, 1> franka_pole::TestPositionController::_get_position_level2(const ros::Time &time, const ros::Duration &period)
{
    _time += period;
    Eigen::Matrix<double, 3, 1> position_target;
    for (size_t i = 0; i < 3; i++)
    {
        double phi = parameters->test_phase(i) + 2 * M_PI * parameters->test_frequency(i) * _time.toSec();
        double a = parameters->test_amplitude(i);
        if (parameters->test_rectangle) position_target(i) = 0.0; //???
        else position_target(i) = a * -sin(phi);
    }
    return parameters->target_effector_position + position_target * std::min(_time.toSec() / parameters->startup_time, 1.0);
}

Eigen::Matrix<double, 3, 1> franka_pole::TestPositionController::_get_velocity_level2(const ros::Time &time, const ros::Duration &period)
{
    Eigen::Matrix<double, 3, 1> velocity_target;
    for (size_t i = 0; i < 3; i++)
    {
        double phi = parameters->test_phase(i) + 2 * M_PI * parameters->test_frequency(i) * _time.toSec();
        double a = 2 * M_PI * parameters->test_frequency(i) * parameters->test_amplitude(i);
        if (parameters->test_rectangle) velocity_target(i) = 0.0; //???
        else velocity_target(i) = a * -cos(phi);
    }
    return velocity_target * std::min(_time.toSec() / parameters->startup_time, 1.0);
}

FRANKA_POLE_CONTROLLER_IMPLEMENTATION(franka_pole::TestPositionController);