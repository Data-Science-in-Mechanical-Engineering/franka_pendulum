#include <franka_pendulum/test_acceleration_controller.h>
#include <franka_pendulum/parameters.h>

bool franka_pendulum::TestAccelerationController::_init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    _time = ros::Time(0,0);
    return true;
}

Eigen::Matrix<double, 3, 1> franka_pendulum::TestAccelerationController::_get_acceleration_level2(const ros::Time &time, const ros::Duration &period)
{
    Eigen::Matrix<double, 3, 1> acceleration_target;
    for (size_t i = 0; i < 3; i++)
    {
        double phi = parameters->test_phase(i) + 2 * M_PI * parameters->test_frequency(i) * _time.toSec();
        double a = pow(2 * M_PI * parameters->test_frequency(i), 2) * parameters->test_amplitude(i);
        if (parameters->test_rectangle) acceleration_target(i) = a * (sin(phi) > 0.0 ? 1.0 : -1.0);
        else acceleration_target(i) = a * sin(phi);
    }

    _time += period;
    return (_time.toSec() > parameters->startup_time) ? (acceleration_target) : (acceleration_target * _time.toSec() / parameters->startup_time);
}

FRANKA_POLE_CONTROLLER_IMPLEMENTATION(franka_pendulum::TestAccelerationController);