#include <franka_pole/publisher.h>
#include <franka_pole/Sample.h>

franka_pole::Publisher::Publisher(Controller *controller, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    _publisher = node_handle.advertise<franka_pole::Sample>("/franka_pole/sample", 10);
    _ok = true;
}

bool franka_pole::Publisher::ok() const
{
    return _ok;
}

void franka_pole::Publisher::publish()
{
    Sample sample;

    sample.pole_timestamp = _pole_timestamp;
    for (size_t i = 0; i < 2; i++) sample.pole_joint_angle[i] = _pole_joint_angle(i);
    for (size_t i = 0; i < 2; i++) sample.pole_joint_dangle[i] = _pole_joint_dangle(i);
    for (size_t i = 0; i < 2; i++) sample.pole_angle[i] = _pole_angle(i);

    sample.franka_timestamp = _franka_timestamp;
    for (size_t i = 0; i < 3; i++) sample.franka_effector_position[i] = _franka_effector_position(i);
    for (size_t i = 0; i < 3; i++) sample.franka_effector_velocity[i] = _franka_effector_velocity(i);

    sample.command_timestamp = _command_timestamp;
    for (size_t i = 0; i < 3; i++) sample.command_effector_position[i] = _command_effector_position(i);
    for (size_t i = 0; i < 3; i++) sample.command_effector_velocity[i] = _command_effector_velocity(i);
    for (size_t i = 0; i < 3; i++) sample.command_effector_acceleration[i] = _command_effector_acceleration(i);

    sample.reset = _reset;

    _publisher.publish(sample);
}

void franka_pole::Publisher::set_franka_timestamp(const ros::Time &timestamp)
{
    _franka_timestamp = timestamp.toSec();
}

void franka_pole::Publisher::set_franka_effector_position(const Eigen::Matrix<double, 3, 1> &position)
{
    _franka_effector_position = position;
}

void franka_pole::Publisher::set_franka_effector_velocity(const Eigen::Matrix<double, 3, 1> &velocity)
{
    _franka_effector_velocity = velocity;
}

void franka_pole::Publisher::set_franka_effector_orientation(const Eigen::Quaterniond &orientation)
{
    _franka_effector_orientation = orientation;
}

void franka_pole::Publisher::set_pole_timestamp(const ros::Time &timestamp)
{
    _pole_timestamp = timestamp.toSec();
}

void franka_pole::Publisher::set_pole_joint_angle(const Eigen::Matrix<double, 2, 1> &angle)
{
    _pole_joint_angle = angle;
}

void franka_pole::Publisher::set_pole_joint_dangle(const Eigen::Matrix<double, 2, 1> &dangle)
{
    _pole_joint_dangle = dangle;
}

void franka_pole::Publisher::set_pole_angle(const Eigen::Matrix<double, 2, 1> &angle)
{
    _pole_angle = angle;
}

void franka_pole::Publisher::set_command_timestamp(const ros::Time &timestamp)
{
    _command_timestamp = timestamp.toSec();
}

void franka_pole::Publisher::set_command_effector_position(const Eigen::Matrix<double, 3, 1> &position)
{
    _command_effector_position = position;
}

void franka_pole::Publisher::set_command_effector_velocity(const Eigen::Matrix<double, 3, 1> &velocity)
{
    _command_effector_velocity = velocity;
}
void franka_pole::Publisher::set_command_effector_acceleration(const Eigen::Matrix<double, 3, 1> &acceleration)
{
    _command_effector_acceleration = acceleration;
}

void franka_pole::Publisher::set_reset(bool reset)
{
    _reset = reset;
}