#include <franka_pendulum/simple_position_controller.h>
#include <franka_pendulum/parameters.h>
#include <franka_pendulum/franka_state.h>
#include <franka_pendulum/pendulum_state.h>

bool franka_pendulum::SimplePositionController::_init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    _time = ros::Time(0,0);
    return true;
}

Eigen::Matrix<double, 3, 1> franka_pendulum::SimplePositionController::_get_position_level2(const ros::Time &time, const ros::Duration &period)
{
    Eigen::Matrix<double, 3, 1> position_target = Eigen::Matrix<double, 3, 1>::Zero();
    if (pendulum_state != nullptr)
    {
        Eigen::Matrix<double, 4, 1> input = Eigen::Matrix<double, 4, 1>::Zero();

        input(0) = pendulum_state->get_angle()(1);
        input(1) = pendulum_state->get_dangle()(1);
        input(2) = franka_state->get_effector_position()(0) - parameters->target_effector_position(0);
        input(3) = franka_state->get_effector_velocity()(0);
        position_target(0) = parameters->pendulum_control.segment<4>(0).transpose() * input;
            
        input(0) = pendulum_state->get_angle()(0);
        input(1) = pendulum_state->get_dangle()(0);
        input(2) = franka_state->get_effector_position()(1) - parameters->target_effector_position(1);
        input(3) = franka_state->get_effector_velocity()(1);
        position_target(1) = parameters->pendulum_control.segment<4>(4).transpose() * input;
    }

    _time += period;
    return parameters->target_effector_position + ((_time.toSec() > parameters->startup_time) ? (position_target) : (position_target * _time.toSec() / parameters->startup_time));
}

Eigen::Matrix<double, 3, 1> franka_pendulum::SimplePositionController::_get_velocity_level2(const ros::Time &time, const ros::Duration &period)
{
    return Eigen::Matrix<double, 3, 1>::Zero();
}

FRANKA_POLE_CONTROLLER_IMPLEMENTATION(franka_pendulum::SimplePositionController);