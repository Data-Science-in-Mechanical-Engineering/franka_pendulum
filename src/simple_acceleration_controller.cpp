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
    _model = parameters.model();
    _target_position = parameters.target_effector_position();

    _command_subscriber = node_handle.subscribe("/franka_pole/command_parameters", 10, &SimpleAccelerationController::_command_callback, this);

    if (_model == Model::D1)
    {
        const double lqr[4] = { 52.23421734, 14.21253046, 10.0, 17.36721051 };
        _a = std::array<double, 2>({{ 0.0, lqr[0] }});
        _b = std::array<double, 2>({{ 0.0, lqr[1] }});
        _c = std::array<double, 2>({{ 0.0, lqr[2] }});
        _d = std::array<double, 2>({{ 0.0, lqr[3] }});
    }
    else if (_model == Model::D2)
    {
        const double lqr1[4] = { 6.69280164e+01, 1.94527670e+01, 1.41421356e+01, 2.50387242e+01 };
        const double lqr2[4] = { 5.16509316e+01, 1.42812800e+01, 1.00000000e+01, 1.74816219e+01 };
        _a = std::array<double, 2>({{ lqr1[0], lqr2[0] }});
        _b = std::array<double, 2>({{ lqr1[1], lqr2[1] }});
        _c = std::array<double, 2>({{ lqr1[2], lqr2[2] }});
        _d = std::array<double, 2>({{ lqr1[3], lqr2[3] }});
    }
    else
    {
        const double lqr1[4] = { 1.03158335e+02, 2.80403213e+01, 1.41421356e+01, 2.34874244e+01 };
        const double lqr2[4] = { 7.71037535e+01, 2.06923784e+01, 1.00000000e+01, 1.67572449e+01 };
        _a = std::array<double, 2>({{ lqr1[0], lqr2[0] }});
        _b = std::array<double, 2>({{ lqr1[1], lqr2[1] }});
        _c = std::array<double, 2>({{ lqr1[2], lqr2[2] }});
        _d = std::array<double, 2>({{ lqr1[3], lqr2[3] }});
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
    
    if (_model == Model::D2 || _model == Model::D2b) acceleration_target(0) =
        (_a[0] * pole_state->get_angle()(1) +
        _b[0] * pole_state->get_dangle()(1) +
        _c[0] * (franka_state->get_effector_position()(0) - _target_position(0)) +
        _d[0] * franka_state->get_effector_velocity()(0));
        
    acceleration_target(1) =
        (_a[1] * pole_state->get_angle()(0) +
        _b[1] * pole_state->get_dangle()(0) +
        _c[1] * (franka_state->get_effector_position()(1) - _target_position(1)) +
        _d[1] * franka_state->get_effector_velocity()(1));
    
    AccelerationController::_controller_post_update(time, period, acceleration_target);
}

PLUGINLIB_EXPORT_CLASS(franka_pole::SimpleAccelerationController, controller_interface::ControllerBase)