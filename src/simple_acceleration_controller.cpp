#include <franka_pole/simple_acceleration_controller.h>
#include <franka_pole/parameters.h>
#include <pluginlib/class_list_macros.h>

void franka_pole::SimpleAccelerationController::_command_callback(const franka_pole::CommandParameters::ConstPtr &msg)
{
    for (size_t i = 0; i < 2; i++)
    {
        _a[i] = msg->a[i];
        _b[i] = msg->b[i];
        _c[i] = msg->c[i];
        _d[i] = msg->d[i];
    }
}

bool franka_pole::SimpleAccelerationController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    if (!AccelerationController::_controller_init(robot_hw, node_handle)) return false;

    Parameters parameters(node_handle);
    _two_dimensional = parameters.two_dimensional();
    _target_position = parameters.target_effector_position();

    _command_subscriber = node_handle.subscribe("/franka_pole/command_parameters", 10, &SimpleAccelerationController::_command_callback, this);

    if (_two_dimensional)
    {
        const double lqr1[4] = { 4.24364503e+01, 1.27280778e+01, 1.00000000e+01, 1.84860853e+01 };
        const double lqr2[4] = { 1.85688832e+01, 4.62233425e+00, 3.16227766e+00, 5.85610013e+00 };
        _a = std::array<double, 2>({{ lqr1[0], lqr2[0] }});
        _b = std::array<double, 2>({{ lqr1[1], lqr2[1] }});
        _c = std::array<double, 2>({{ lqr1[2], lqr2[2] }});
        _d = std::array<double, 2>({{ lqr1[3], lqr2[3] }});
    }
    else
    {
        const double lqr[4] = { 20.84313016, 5.06703731, 3.16227766, 5.76745694 };
        _a = std::array<double, 2>({{ 0.0, lqr[0] }});
        _b = std::array<double, 2>({{ 0.0, lqr[1] }});
        _c = std::array<double, 2>({{ 0.0, lqr[2] }});
        _d = std::array<double, 2>({{ 0.0, lqr[3] }});
    }

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
    
    if (_two_dimensional) acceleration_target(0) =
        (_a[0] * pole_state->get_angle()(1) +
        _b[0] * pole_state->get_joint_dangle()(1) +
        _c[0] * (franka_state->get_effector_position()(0) - _target_position(0)) +
        _d[0] * franka_state->get_effector_velocity()(0));
        
    acceleration_target(1) =
        (_a[1] * pole_state->get_angle()(0) +
        _b[1] * pole_state->get_joint_dangle()(0) +
        _c[1] * (franka_state->get_effector_position()(1) - _target_position(1)) +
        _d[1] * franka_state->get_effector_velocity()(1));
    
    AccelerationController::_controller_post_update(time, period, acceleration_target);
}

PLUGINLIB_EXPORT_CLASS(franka_pole::SimpleAccelerationController, controller_interface::ControllerBase)