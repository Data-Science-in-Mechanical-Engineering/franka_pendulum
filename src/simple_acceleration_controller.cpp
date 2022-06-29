#include <franka_pole/simple_acceleration_controller.h>
#include <pluginlib/class_list_macros.h>

void franka_pole::SimpleAccelerationController::_command_callback(const franka_pole::CommandParameters::ConstPtr &msg)
{
    _a = msg->a;
    _b = msg->b;
    _c = msg->c;
    _d = msg->d;
}

bool franka_pole::SimpleAccelerationController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    if (!AccelerationController::_controller_init(robot_hw, node_handle)) return false;

    _command_subscriber = node_handle.subscribe("/franka_pole/parameters_command", 10, &SimpleAccelerationController::_command_callback, this);

    return true;
}

void franka_pole::SimpleAccelerationController::starting(const ros::Time &time)
{
    AccelerationController::_controller_starting(time);
}

void franka_pole::SimpleAccelerationController::update(const ros::Time &time, const ros::Duration &period)
{
    AccelerationController::_controller_pre_update(time, period);

    Eigen::Matrix<double, 3, 1> acceleration_target = Eigen::Matrix<double, 3, 1>::Zero();
    
    acceleration_target(1) =
        _a * pole_state->get_angle()(0) +
        _b * pole_state->get_dangle()(0) +
        _c * franka_state->get_effector_position()(1) +
        _d * franka_state->get_effector_velocity()(1);
    
    if (param->two_dimensional()) acceleration_target(0) =
        _a * pole_state->get_angle()(1) +
        _b * pole_state->get_dangle()(1) +
        _c * franka_state->get_effector_position()(0) +
        _d * franka_state->get_effector_velocity()(0);
    
    AccelerationController::_controller_post_update(time, period, acceleration_target);
}

PLUGINLIB_EXPORT_CLASS(franka_pole::SimpleAccelerationController, controller_interface::ControllerBase)