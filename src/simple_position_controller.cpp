#include <pinocchio/fwd.hpp>
#include <franka_pole/simple_position_controller.h>
#include <franka_pole/parameters.h>
#include <pluginlib/class_list_macros.h>

void franka_pole::SimplePositionController::_command_callback(const franka_pole::CommandParameters::ConstPtr &msg)
{
    for (size_t i = 0; i < 2; i++)
    {
        _a[i] = msg->a[i];
        _b[i] = msg->b[i];
        _c[i] = msg->c[i];
        _d[i] = msg->d[i];
    }
}

bool franka_pole::SimplePositionController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    if (!PositionController::_controller_init(robot_hw, node_handle)) return false;

    Parameters parameters(node_handle);
    _two_dimensional = parameters.two_dimensional();
    _target_position = parameters.target_effector_position();
    _max_effector_position = parameters.max_effector_position();
    _min_effector_position = parameters.min_effector_position();

    _command_subscriber = node_handle.subscribe("/franka_pole/command_parameters", 10, &SimplePositionController::_command_callback, this);

    _a = std::array<double, 2>({{ 16.363880157470703 / 30, 16.363880157470703 / 30 }});
    _b = std::array<double, 2>({{ 9.875003814697266 / 30, 9.875003814697266 / 30 }});
    _c = std::array<double, 2>({{ 7.015979766845703 / 30, 7.015979766845703 / 30 }});
    _d = std::array<double, 2>({{ 11.86760425567627 / 30, 11.86760425567627 / 30 }});

    return true;
}

void franka_pole::SimplePositionController::starting(const ros::Time &time)
{
    PositionController::_controller_starting(time);
}

void franka_pole::SimplePositionController::update(const ros::Time &time, const ros::Duration &period)
{
    PositionController::_controller_pre_update(time, period);

    Eigen::Matrix<double, 3, 1> position_target = franka_state->get_effector_position();

    position_target(1) +=
        (_a[1] * pole_state->get_angle()(0) +
        _b[1] * pole_state->get_joint_dangle()(0) +
        _c[1] * (franka_state->get_effector_position()(1) - _target_position(1)) +
        _d[1] * franka_state->get_effector_velocity()(1));
    

    if (_two_dimensional) position_target(0) +=
        (_a[0] * pole_state->get_angle()(1) +
        _b[0] * pole_state->get_joint_dangle()(1) +
        _c[0] * (franka_state->get_effector_position()(0) - _target_position(0)) +
        _d[0] * franka_state->get_effector_velocity()(0));
    
    for (size_t i = 0; i < 3; i++)
    {
        if (position_target(i) < _min_effector_position(i)) position_target(i) = _min_effector_position(i);
        else if (position_target(i) > _max_effector_position(i)) position_target(i) = _max_effector_position(i);
    }
    
    PositionController::_controller_post_update(time, period, position_target, Eigen::Matrix<double, 3, 1>::Zero());
}

PLUGINLIB_EXPORT_CLASS(franka_pole::SimplePositionController, controller_interface::ControllerBase)