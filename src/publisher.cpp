#include <franka_pole/publisher.h>

franka_pole::Publisher::Publisher(ros::NodeHandle &node_handle)
{
    _publisher = node_handle.advertise<franka_pole::Sample>("/franka_pole/sample", 10);
    
    set_franka_timestamp(ros::Time(0.0));
    set_franka_effector_position(Eigen::Matrix<double, 3, 1>::Zero());
    set_franka_effector_velocity(Eigen::Matrix<double, 3, 1>::Zero());
    set_franka_effector_orientation(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0));
    set_pole_timestamp(ros::Time(0.0));
    set_pole_angle(Eigen::Matrix<double, 2, 1>::Zero());
    set_pole_dangle(Eigen::Matrix<double, 2, 1>::Zero());
    set_pole_joint_angle(Eigen::Matrix<double, 2, 1>::Zero());
    set_command_timestamp(ros::Time(0.0));
    set_command_effector_position(Eigen::Matrix<double, 3, 1>::Zero());
    set_command_effector_velocity(Eigen::Matrix<double, 3, 1>::Zero());
    set_command_effector_acceleration(Eigen::Matrix<double, 3, 1>::Zero());
    set_reset(false);
}

void franka_pole::Publisher::publish()
{
    _publisher.publish(_sample);
}

void franka_pole::Publisher::set_franka_timestamp(const ros::Time &timestamp)
{
    _sample.franka_timestamp = timestamp.toSec();
}

void franka_pole::Publisher::set_franka_effector_position(const Eigen::Matrix<double, 3, 1> &position)
{
    Eigen::Matrix<double, 3, 1>::Map(&_sample.franka_effector_position[0]) = position;
}

void franka_pole::Publisher::set_franka_effector_velocity(const Eigen::Matrix<double, 3, 1> &velocity)
{
    Eigen::Matrix<double, 3, 1>::Map(&_sample.franka_effector_velocity[0]) = velocity;
}

void franka_pole::Publisher::set_franka_effector_orientation(const Eigen::Quaterniond &orientation)
{
}

void franka_pole::Publisher::set_pole_timestamp(const ros::Time &timestamp)
{
    _sample.pole_timestamp = timestamp.toSec();
}

void franka_pole::Publisher::set_pole_angle(const Eigen::Matrix<double, 2, 1> &angle)
{
    Eigen::Matrix<double, 2, 1>::Map(&_sample.pole_angle[0]) = angle;
}

void franka_pole::Publisher::set_pole_dangle(const Eigen::Matrix<double, 2, 1> &dangle)
{
    Eigen::Matrix<double, 2, 1>::Map(&_sample.pole_dangle[0]) = dangle;
}

void franka_pole::Publisher::set_pole_joint_angle(const Eigen::Matrix<double, 2, 1> &angle)
{
    Eigen::Matrix<double, 2, 1>::Map(&_sample.pole_joint_angle[0]) = angle;
}

void franka_pole::Publisher::set_pole_joint_dangle(const Eigen::Matrix<double, 2, 1> &dangle)
{
    Eigen::Matrix<double, 2, 1>::Map(&_sample.pole_joint_dangle[0]) = dangle;
}

void franka_pole::Publisher::set_command_timestamp(const ros::Time &timestamp)
{
    _sample.command_timestamp = timestamp.toSec();
}

void franka_pole::Publisher::set_command_effector_position(const Eigen::Matrix<double, 3, 1> &position)
{
    Eigen::Matrix<double, 3, 1>::Map(&_sample.command_effector_position[0]) = position;
}

void franka_pole::Publisher::set_command_effector_velocity(const Eigen::Matrix<double, 3, 1> &velocity)
{
    Eigen::Matrix<double, 3, 1>::Map(&_sample.command_effector_velocity[0]) = velocity;
}
void franka_pole::Publisher::set_command_effector_acceleration(const Eigen::Matrix<double, 3, 1> &acceleration)
{
    Eigen::Matrix<double, 3, 1>::Map(&_sample.command_effector_acceleration[0]) = acceleration;
}

void franka_pole::Publisher::set_reset(bool reset)
{
    _sample.reset = reset;
}