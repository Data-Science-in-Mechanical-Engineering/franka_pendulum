#include <franka_pole/acceleration_controller.h>
#include <franka_pole/parameters.h>
#include <franka_pole/franka_state.h>
#include <franka_pole/pole_state.h>
#include <franka_pole/CommandAcceleration.h>

#include <Eigen/Dense>

namespace franka_pole_example
{
    class HybridController : public franka_pole::AccelerationController
    {
    private:
        //Subscriber
        Eigen::Matrix<double, 3, 1> _acceleration_target;
        ros::Subscriber _subscriber;
        void _callback(const franka_pole::CommandAcceleration::ConstPtr &msg);

        //Overrides from franka_pole::AccelerationController
        bool _init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        Eigen::Matrix<double, 3, 1> _get_acceleration_level2(const ros::Time &time, const ros::Duration &period) override;

        FRANKA_POLE_CONTROLLER_DECLARATION();
    };
}

void franka_pole_example::HybridController::_callback(const franka_pole::CommandAcceleration::ConstPtr &msg)
{
    std::lock_guard<std::mutex> guard(mutex);
    _acceleration_target = Eigen::Matrix<double, 3, 1>::Map(&msg->command_effector_acceleration[0]);
}

bool franka_pole_example::HybridController::_init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    _acceleration_target = Eigen::Matrix<double, 3, 1>::Zero();
    _subscriber = node_handle.subscribe("/" + parameters->namespacee + "/command_acceleration", 10, &HybridController::_callback, this);
    return true;
}

Eigen::Matrix<double, 3, 1> franka_pole_example::HybridController::_get_acceleration_level2(const ros::Time &time, const ros::Duration &period)
{
    Eigen::Matrix<double, 3, 1> acceleration_target = Eigen::Matrix<double, 3, 1>::Zero();
    if (pole_state != nullptr)
    {
        Eigen::Matrix<double, 4, 1> input = Eigen::Matrix<double, 4, 1>::Zero();

        input(0) = pole_state->get_angle()(1);
        input(1) = pole_state->get_dangle()(1);
        input(2) = franka_state->get_effector_position()(0) - parameters->target_effector_position(0);
        input(3) = franka_state->get_effector_velocity()(0);
        acceleration_target(0) = parameters->control.segment<4>(0).transpose() * input;
            
        input(0) = pole_state->get_angle()(0);
        input(1) = pole_state->get_dangle()(0);
        input(2) = franka_state->get_effector_position()(1) - parameters->target_effector_position(1);
        input(3) = franka_state->get_effector_velocity()(1);
        acceleration_target(1) = parameters->control.segment<4>(4).transpose() * input;
    }

    return acceleration_target + _acceleration_target;
}

FRANKA_POLE_CONTROLLER_IMPLEMENTATION(franka_pole_example::HybridController);