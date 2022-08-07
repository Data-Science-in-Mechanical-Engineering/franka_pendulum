#include <franka_pole/simple_position_controller.h>
#include <franka_pole/parameters.h>
#include <franka_pole/franka_state.h>
#include <franka_pole/pole_state.h>

bool franka_pole::SimplePositionController::_init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    return true;
}

Eigen::Matrix<double, 3, 1> franka_pole::SimplePositionController::_get_position_level2(const ros::Time &time, const ros::Duration &period)
{
    Eigen::Matrix<double, 3, 1> position_target = Eigen::Matrix<double, 3, 1>::Zero();
    if (pole_state != nullptr)
    {
        Eigen::Matrix<double, 4, 1> input = Eigen::Matrix<double, 4, 1>::Zero();

        input(0) = pole_state->get_angle()(1);
        input(1) = pole_state->get_dangle()(1);
        input(2) = franka_state->get_effector_position()(0) - parameters->target_effector_position(0);
        input(3) = franka_state->get_effector_velocity()(0);
        position_target(0) = parameters->control.segment<4>(0).transpose() * input;
            
        input(0) = pole_state->get_angle()(0);
        input(1) = pole_state->get_dangle()(0);
        input(2) = franka_state->get_effector_position()(1) - parameters->target_effector_position(1);
        input(3) = franka_state->get_effector_velocity()(1);
        position_target(1) = parameters->control.segment<4>(4).transpose() * input;
    }

    return position_target;
}

Eigen::Matrix<double, 3, 1> franka_pole::SimplePositionController::_get_velocity_level2(const ros::Time &time, const ros::Duration &period)
{
    return Eigen::Matrix<double, 3, 1>::Zero();
}

FRANKA_POLE_CONTROLLER_IMPLEMENTATION(franka_pole::SimplePositionController);