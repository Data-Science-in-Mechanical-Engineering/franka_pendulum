#include <franka_pole/simple_position_controller.h>
#include <franka_pole/pole_state.h>
#include <franka_pole/franka_state.h>
#include <pluginlib/class_list_macros.h>

void franka_pole::SimplePositionController::_command_callback(const franka_pole::CommandParameters::ConstPtr &msg)
{
    _a = msg->a;
    _b = msg->b;
    _c = msg->c;
    _d = msg->d;
}

bool franka_pole::SimplePositionController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    if (!PositionController::_controller_init(robot_hw, node_handle)) return false;

    _command_subscriber = node_handle.subscribe("/franka_pole/parameters_command", 10, &SimplePositionController::_command_callback, this);

    return true;
}

void franka_pole::SimplePositionController::starting(const ros::Time &time)
{
    PositionController::_controller_starting(time);
}

void franka_pole::SimplePositionController::update(const ros::Time &time, const ros::Duration &period)
{
    PositionController::_controller_pre_update(time, period);

    Eigen::Matrix<double, 3, 1> position_target = get_box_center();
    position_target(1) += (_a * pole_state->get_angle()(0) + _b * pole_state->get_dangle()(0) + _c * (franka_state->get_effector_position()(1) - get_box_center()(1)) + _d * franka_state->get_effector_velocity()(1));
    if (is_two_dimensional())
    {
        position_target(0) += (_a * pole_state->get_angle()(1) + _b * pole_state->get_dangle()(1) + _c * (franka_state->get_effector_position()(0) - get_box_center()(0)) + _d * franka_state->get_effector_velocity()(0));
    }
    
    for (size_t i = 0; i < 3; i++)
    {
        if (position_target(i) < get_box_min()(i)) position_target(i) = get_box_min()(i);
        else if (position_target(i) > get_box_max()(i)) position_target(i) = get_box_max()(i);
    }
    PositionController::_controller_post_update(time, period, position_target, Eigen::Matrix<double, 3, 1>::Zero());
}

PLUGINLIB_EXPORT_CLASS(franka_pole::SimplePositionController, controller_interface::ControllerBase)