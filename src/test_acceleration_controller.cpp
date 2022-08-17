#include <franka_pole/test_acceleration_controller.h>
#include <franka_pole/parameters.h>

bool franka_pole::TestAccelerationController::_init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    _time = ros::Time(0,0);
    return true;
}

Eigen::Matrix<double, 3, 1> franka_pole::TestAccelerationController::_get_acceleration_level2(const ros::Time &time, const ros::Duration &period)
{
    _time += period;
    Eigen::Matrix<double, 3, 1> acceleration_target;
    for (size_t i = 0; i < 3; i++)
    {
        double phi = parameters->test_phase(i) + 2 * M_PI * parameters->test_frequency(i) * _time.toSec();
        double a = pow(2 * M_PI * parameters->test_frequency(i), 2) * parameters->test_amplitude(i);
        if (parameters->test_rectangle) acceleration_target(i) = a * (sin(phi) > 0.0 ? 1.0 : -1.0);
        else acceleration_target(i) = a * sin(phi);
    }
    return acceleration_target * std::min(_time.toSec() / parameters->startup_time, 1.0);
}

FRANKA_POLE_CONTROLLER_IMPLEMENTATION(franka_pole::TestAccelerationController);