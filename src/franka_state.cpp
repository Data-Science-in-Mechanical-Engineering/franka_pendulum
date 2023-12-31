#include <franka_pendulum/franka_model.h>
#include <franka_pendulum/franka_state.h>
#include <franka_pendulum/parameters.h>
#include <franka_pendulum/publisher.h>
#include <time.h>

franka_pendulum::FrankaState::FrankaState(const Parameters *parameters, FrankaModel *franka_model, Publisher *publisher, hardware_interface::RobotHW *robot_hw) :
_parameters(parameters), _franka_model(franka_model), _publisher(publisher), _timestamp(0.0)
{
    #ifdef FRANKA_POLE_VELOCITY_INTERFACE
        //State interface
        auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) throw std::runtime_error("franka_pendulum::FrankaState::FrankaState(): Error getting state interface from hardware");
        _state_handle = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(_parameters->arm_id + "_robot"));

        //Velocity interface
        auto *velocity_interface = robot_hw->get<franka_hw::FrankaVelocityCartesianInterface>();
        if (velocity_interface == nullptr) throw std::runtime_error("franka_pendulum::FrankaState::FrankaState(): Error getting cartesian velocity interface from hardware");
        _velocity_handle = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(velocity_interface->getHandle(_parameters->arm_id + "_robot"));
    #else
        //Joint handles
        auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr) throw std::runtime_error("franka_pendulum::FrankaState::FrankaState(): Error getting effort joint interface from hardware");
        for (size_t i = 0; i < 7; i++)
        {
            _joint_handles[i] = effort_joint_interface->getHandle(_parameters->arm_id + "_joint" + std::to_string(i + 1));
        }
    #endif
}

void franka_pendulum::FrankaState::reset()
{
    //Random
    for (size_t i = 0; i < 7; i++)
    {
        _random_position_distributions[i] = std::normal_distribution<double>(0.0, _parameters->joint_position_standard_deviation(i));
        _random_velocity_distributions[i] = std::normal_distribution<double>(0.0, _parameters->joint_velocity_standard_deviation(i));
    }
    _random_engine.seed(time(nullptr));

    //Initial readings
    update(ros::Time(0,0));
}

void franka_pendulum::FrankaState::update(const ros::Time &time)
{
    //Measurement
    _timestamp = time.toSec();
    #ifdef FRANKA_POLE_VELOCITY_INTERFACE
        franka::RobotState state = _state_handle->getRobotState();
        for (size_t i = 0; i < 7; i++)
        {
            _exact_joint_positions(i) = state.q[i] + _parameters->joint_position_mean(i);
            _joint_velocities(i) = state.dq[i];
        }
    #else
        for (size_t i = 0; i < 7; i++)
        {
            _exact_joint_positions(i) = _joint_handles[i].getPosition() + _parameters->joint_position_mean(i);
            _joint_velocities(i) = _joint_handles[i].getVelocity();
        }
    #endif
    
    //Adding noise
    if (_parameters->simulated)
    {
        for (size_t i = 0; i < 7; i++)
        {
            _joint_positions(i) = _exact_joint_positions(i) + _random_position_distributions[i](_random_engine);
            _joint_velocities(i) = _joint_velocities(i) + _random_velocity_distributions[i](_random_engine);
        }
    }
    else
    {
        _joint_positions = _exact_joint_positions;
    }
    
    //Basic computations (that can be obtained with StateInterface)
    _effector_position = _franka_model->effector_forward_kinematics(_joint_positions, &_effector_orientation);
    _effector_velocity = _franka_model->get_effector_jacobian(_joint_positions) * _joint_velocities;
    
    //Publish
    _publisher->set_franka(
        time,
        _effector_position,
        _effector_orientation,
        _effector_velocity,
        _joint_positions,
        _joint_velocities);
}

double franka_pendulum::FrankaState::get_timestamp() const
{
    return _timestamp;
}

Eigen::Matrix<double, 7, 1> franka_pendulum::FrankaState::get_joint_positions() const
{
    return _joint_positions;
}

Eigen::Matrix<double, 7, 1> franka_pendulum::FrankaState::get_joint_velocities() const
{
    return _joint_velocities;
}

Eigen::Matrix<double, 3, 1> franka_pendulum::FrankaState::get_effector_position() const
{
    return _effector_position;
}

Eigen::Quaterniond franka_pendulum::FrankaState::get_effector_orientation() const
{
    return _effector_orientation;
}

Eigen::Matrix<double, 6, 1> franka_pendulum::FrankaState::get_effector_velocity() const
{
    return _effector_velocity;
}

#ifndef FRANKA_POLE_VELOCITY_INTERFACE
void franka_pendulum::FrankaState::_set_torque(const Eigen::Matrix<double, 7, 1> &torque)
{
    for (size_t i = 0; i < 7; i++) _joint_handles[i].setCommand(torque(i));
}
#else
void franka_pendulum::FrankaState::_set_velocity(const Eigen::Matrix<double, 6, 1> &velocity)
{
    std::array<double, 6> v;
    Eigen::Matrix<double, 6, 1>::Map(&v[0]) = velocity;
    _velocity_handle->setCommand(v);
}
#endif