#include <franka_pendulum/test_position_controller.h>
#include <franka_pendulum/parameters.h>

bool franka_pendulum::TestPositionController::_init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    _time = ros::Time(0,0);
    return true;
}

Eigen::Matrix<double, 3, 1> franka_pendulum::TestPositionController::_get_position_level2(const ros::Time &time, const ros::Duration &period)
{
    Eigen::Matrix<double, 3, 1> position_target;
    for (size_t i = 0; i < 3; i++)
    {
        double phi = parameters->test_phase(i) + 2 * M_PI * parameters->test_frequency(i) * _time.toSec();
        double a = parameters->test_amplitude(i);
        if (parameters->test_rectangle) position_target(i) = 0.0; //???
        else position_target(i) = a * -sin(phi);
    }

    _time += period;
    return parameters->target_effector_position + ((_time.toSec() > parameters->startup_time) ? (position_target) : (position_target * _time.toSec() / parameters->startup_time));
}

Eigen::Matrix<double, 3, 1> franka_pendulum::TestPositionController::_get_velocity_level2(const ros::Time &time, const ros::Duration &period)
{
    Eigen::Matrix<double, 3, 1> velocity_target;
    for (size_t i = 0; i < 3; i++)
    {
        double phi = parameters->test_phase(i) + 2 * M_PI * parameters->test_frequency(i) * _time.toSec();
        double a = 2 * M_PI * parameters->test_frequency(i) * parameters->test_amplitude(i);
        if (parameters->test_rectangle) velocity_target(i) = 0.0; //???
        else velocity_target(i) = a * -cos(phi);
    }

    return (_time.toSec() > parameters->startup_time) ? (velocity_target) : (velocity_target * _time.toSec() / parameters->startup_time);
}

FRANKA_POLE_CONTROLLER_IMPLEMENTATION(franka_pendulum::TestPositionController);