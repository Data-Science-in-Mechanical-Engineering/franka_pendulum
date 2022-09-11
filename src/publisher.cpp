#include <franka_pendulum/publisher.h>
#include <franka_pendulum/parameters.h>

franka_pendulum::Publisher::Publisher(const Parameters *parameters, ros::NodeHandle &node_handle) :
_parameters(parameters)
{
    _sample_publisher = node_handle.advertise<franka_pendulum::Sample>("/" + _parameters->namespacee + "/sample", 10);
    _joint_state_publisher = node_handle.advertise<sensor_msgs::JointState>("/" + _parameters->namespacee + "/joint_state", 10);
    
    
    if (get_model_freedom(_parameters->model) == 0)
    {
        _joint_state.name.resize(9);
        _joint_state.position.resize(9, 0.0);
        _joint_state.velocity.resize(9, 0.0);
        _joint_state.effort.resize(9, 0.0);
    }
    else if (get_model_freedom(_parameters->model) == 1)
    {
        _joint_state.name.resize(10);
        _joint_state.position.resize(10, 0.0);
        _joint_state.velocity.resize(10, 0.0);
        _joint_state.effort.resize(10, 0.0);
        _joint_state.name[9] = _parameters->arm_id + "_pendulum_joint_x";
    }
    else
    {
        _joint_state.name.resize(11);
        _joint_state.position.resize(11, 0.0);
        _joint_state.velocity.resize(11, 0.0);
        _joint_state.effort.resize(11, 0.0);
        _joint_state.name[9] = _parameters->arm_id + "_pendulum_joint_x";
        _joint_state.name[10] = _parameters->arm_id + "_pendulum_joint_y";
    }
    for (size_t i = 0; i < 7; i++) _joint_state.name[i] = _parameters->arm_id + "_joint" + std::to_string(i+1);
    for (size_t i = 0; i < 2; i++) _joint_state.name[7+i] = _parameters->arm_id + "_finger_joint" + std::to_string(i+1);

    set_pendulum(
        ros::Time(0,0),
        Eigen::Matrix<double, 2, 1>::Zero(),
        Eigen::Matrix<double, 2, 1>::Zero(),
        Eigen::Matrix<double, 2, 1>::Zero(),
        Eigen::Matrix<double, 2, 1>::Zero());

    set_franka(ros::Time(0,0),
        Eigen::Matrix<double, 3, 1>::Zero(),
        Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
        Eigen::Matrix<double, 6, 1>::Zero(),
        Eigen::Matrix<double, 7, 1>::Zero(),
        Eigen::Matrix<double, 7, 1>::Zero());

    set_command(ros::Time(0,0),
        Eigen::Matrix<double, 3, 1>::Zero(),
        Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
        Eigen::Matrix<double, 6, 1>::Zero(),
        Eigen::Matrix<double, 6, 1>::Zero(),
        Eigen::Matrix<double, 7, 1>::Zero(),
        Eigen::Matrix<double, 7, 1>::Zero(),
        Eigen::Matrix<double, 7, 1>::Zero(),
        Eigen::Matrix<double, 7, 1>::Zero());

    set_reset(false);
}

void franka_pendulum::Publisher::publish()
{
    _sample_publisher.publish(_sample);
    _joint_state_publisher.publish(_joint_state);
}

void franka_pendulum::Publisher::set_pendulum(
    const ros::Time &timestamp,
    const Eigen::Matrix<double, 2, 1> &angle,
    const Eigen::Matrix<double, 2, 1> &dangle,
    const Eigen::Matrix<double, 2, 1> &joint_angle,
    const Eigen::Matrix<double, 2, 1> &joint_dangle)
{
    _sample.pendulum_timestamp = timestamp.toSec();
    Eigen::Matrix<double, 2, 1>::Map(&_sample.pendulum_angle[0]) = angle;
    Eigen::Matrix<double, 2, 1>::Map(&_sample.pendulum_dangle[0]) = dangle;

    Eigen::Matrix<double, 2, 1>::Map(&_sample.pendulum_joint_angle[0]) = angle;
    if (get_model_freedom(_parameters->model) >= 1) _joint_state.position[9] = -angle(0);
    if (get_model_freedom(_parameters->model) == 2) _joint_state.position[10] = angle(1);

    Eigen::Matrix<double, 2, 1>::Map(&_sample.pendulum_joint_dangle[0]) = dangle;
    if (get_model_freedom(_parameters->model) >= 1) _joint_state.velocity[9] = -dangle(0);
    if (get_model_freedom(_parameters->model) == 2) _joint_state.velocity[10] = dangle(1);
}

void franka_pendulum::Publisher::set_franka(const ros::Time &timestamp,
    const Eigen::Matrix<double, 3, 1> &position,
    const Eigen::Quaterniond &orientation,
    const Eigen::Matrix<double, 6, 1> &velocity,
    const Eigen::Matrix<double, 7, 1> &positions,
    const Eigen::Matrix<double, 7, 1> &velocities)
{
    _sample.franka_timestamp = timestamp.toSec();
    Eigen::Matrix<double, 3, 1>::Map(&_sample.franka_effector_position[0]) = position;
    //Eigen::Matrix<double, 4, 1>::Map(&_sample.franka_effector_orientation[0]) = Eigen::Matrix<double, 4, 1>(orientation.w(), orientation.x(), orientation.y(), orientation.z());
    Eigen::Matrix<double, 3, 1>::Map(&_sample.franka_effector_velocity[0]) = velocity.segment<3>(0);

    Eigen::Matrix<double, 7, 1>::Map(&_sample.franka_joint_positions[0]) = positions;
    Eigen::Matrix<double, 7, 1>::Map(&_joint_state.position[0]) = positions;

    Eigen::Matrix<double, 7, 1>::Map(&_sample.franka_joint_velocities[0]) = velocities;
    Eigen::Matrix<double, 7, 1>::Map(&_joint_state.velocity[0]) = velocities;
}

void franka_pendulum::Publisher::set_command(const ros::Time &timestamp,
    const Eigen::Matrix<double, 3, 1> &position,
    const Eigen::Quaterniond &orientation,
    const Eigen::Matrix<double, 6, 1> &velocity,
    const Eigen::Matrix<double, 6, 1> &acceleration,
    const Eigen::Matrix<double, 7, 1> &positions,
    const Eigen::Matrix<double, 7, 1> &velocities,
    const Eigen::Matrix<double, 7, 1> &accelerations,
    const Eigen::Matrix<double, 7, 1> &torques)
{
    _sample.command_timestamp = timestamp.toSec();
    Eigen::Matrix<double, 3, 1>::Map(&_sample.command_effector_position[0]) = position;
    //Eigen::Matrix<double, 4, 1>::Map(&_sample.franka_effector_orientation[0]) = Eigen::Matrix<double, 4, 1>(orientation.w(), orientation.x(), orientation.y(), orientation.z());
    Eigen::Matrix<double, 3, 1>::Map(&_sample.command_effector_velocity[0]) = velocity.segment<3>(0);
    Eigen::Matrix<double, 3, 1>::Map(&_sample.command_effector_acceleration[0]) = acceleration.segment<3>(0);
    Eigen::Matrix<double, 7, 1>::Map(&_sample.command_joint_positions[0]) = positions;
    Eigen::Matrix<double, 7, 1>::Map(&_sample.command_joint_velocities[0]) = velocities;
    Eigen::Matrix<double, 7, 1>::Map(&_sample.command_joint_accelerations[0]) = accelerations;
    //Eigen::Matrix<double, 7, 1>::Map(&_sample.command_joint_torques[0]) = torques;
    Eigen::Matrix<double, 7, 1>::Map(&_joint_state.effort[0]) = torques;
}

void franka_pendulum::Publisher::set_reset(bool reset)
{
    _sample.reset = reset;
}