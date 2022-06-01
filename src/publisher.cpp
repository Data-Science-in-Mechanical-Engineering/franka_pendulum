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
    sample.pole_angle = _pole_angle;
    sample.pole_dangle = _pole_dangle;

    sample.franka_timestamp = _franka_timestamp;
    sample.franka_effector_y = _franka_effector_position(1);
    sample.franka_effector_dy = _franka_effector_velocity(1);

    sample.control_effector_ddy = _control_effector_acceleration(1);

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

void franka_pole::Publisher::set_pole_angle(double angle)
{
    _pole_angle = angle;
}

void franka_pole::Publisher::set_pole_dangle(double dangle)
{
    _pole_dangle = dangle;
}

void franka_pole::Publisher::set_control_timestamp(const ros::Time &timestamp)
{
    _control_timestamp = timestamp.toSec();
}

void franka_pole::Publisher::set_control_effector_position(const Eigen::Matrix<double, 3, 1> &position)
{
    _control_effector_position = position;
}

void franka_pole::Publisher::set_control_effector_velocity(const Eigen::Matrix<double, 3, 1> &velocity)
{
    _control_effector_velocity = velocity;
}
void franka_pole::Publisher::set_control_effector_acceleration(const Eigen::Matrix<double, 3, 1> &acceleration)
{
    _control_effector_acceleration = acceleration;
}