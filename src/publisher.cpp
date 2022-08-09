#include <franka_pole/publisher.h>
#include <franka_pole/parameters.h>

franka_pole::Publisher::Publisher(const Parameters *parameters, ros::NodeHandle &node_handle) :
_parameters(parameters)
{
    _sample_publisher = node_handle.advertise<franka_pole::Sample>("/franka_pole/sample", 10);
    _joint_state_publisher = node_handle.advertise<sensor_msgs::JointState>("/franka_pole/joint_state", 10);
    
    
    if (_parameters->model == Model::D0)
    {
        _joint_state.name.resize(9);
        _joint_state.position.resize(9, 0.0);
        _joint_state.velocity.resize(9, 0.0);
        _joint_state.effort.resize(9, 0.0);
    }
    else if (_parameters->model == Model::D1)
    {
        _joint_state.name.resize(10);
        _joint_state.position.resize(10, 0.0);
        _joint_state.velocity.resize(10, 0.0);
        _joint_state.effort.resize(10, 0.0);
        _joint_state.name[9] = _parameters->arm_id + "_pole_joint_x";
    }
    else if (_parameters->model == Model::D2 || _parameters->model == Model::D2b)
    {
        _joint_state.name.resize(11);
        _joint_state.position.resize(11, 0.0);
        _joint_state.velocity.resize(11, 0.0);
        _joint_state.effort.resize(11, 0.0);
        _joint_state.name[9] = _parameters->arm_id + "_pole_joint_x";
        _joint_state.name[10] = _parameters->arm_id + "_pole_joint_y";
    }
    for (size_t i = 0; i < 7; i++) _joint_state.name[i] = _parameters->arm_id + "_joint" + std::to_string(i+1);
    for (size_t i = 0; i < 2; i++) _joint_state.name[7+i] = _parameters->arm_id + "_finger_joint" + std::to_string(i+1);

    set_franka_timestamp(ros::Time(0.0));
    set_franka_joint_positions(Eigen::Matrix<double, 7, 1>::Zero());
    set_franka_joint_velocities(Eigen::Matrix<double, 7, 1>::Zero());
    set_franka_joint_torques(Eigen::Matrix<double, 7, 1>::Zero());
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
    _sample_publisher.publish(_sample);
    _joint_state_publisher.publish(_joint_state);
}

void franka_pole::Publisher::set_franka_timestamp(const ros::Time &timestamp)
{
    _sample.franka_timestamp = timestamp.toSec();
}

void franka_pole::Publisher::set_franka_joint_positions(const Eigen::Matrix<double, 7, 1> &positions)
{
    Eigen::Matrix<double, 7, 1>::Map(&_joint_state.position[0]) = positions;
}

void franka_pole::Publisher::set_franka_joint_velocities(const Eigen::Matrix<double, 7, 1> &velocities)
{
    Eigen::Matrix<double, 7, 1>::Map(&_joint_state.velocity[0]) = velocities;
}

void franka_pole::Publisher::set_franka_joint_torques(const Eigen::Matrix<double, 7, 1> &torques)
{
    Eigen::Matrix<double, 7, 1>::Map(&_joint_state.effort[0]) = torques;
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
    if (_parameters->model == Model::D1 || _parameters->model == Model::D2 || _parameters->model == Model::D2b) _joint_state.position[9] = -angle(0);
    if (_parameters->model == Model::D2 || _parameters->model == Model::D2b) _joint_state.position[10] = angle(1);
}

void franka_pole::Publisher::set_pole_joint_dangle(const Eigen::Matrix<double, 2, 1> &dangle)
{
    Eigen::Matrix<double, 2, 1>::Map(&_sample.pole_joint_dangle[0]) = dangle;
    if (_parameters->model == Model::D1 || _parameters->model == Model::D2 || _parameters->model == Model::D2b) _joint_state.velocity[9] = -dangle(0);
    if (_parameters->model == Model::D2 || _parameters->model == Model::D2b) _joint_state.velocity[10] = dangle(1);
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