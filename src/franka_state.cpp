#include <pinocchio/fwd.hpp>
#include <franka_pole/franka_state.h>
#include <franka_pole/controller.h>
#include <franka_pole/parameters.h>
#include <franka_pole/publisher.h>
#include <time.h>

franka_pole::FrankaState::FrankaState(Controller *controller, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) : _controller(controller)
{
    Parameters parameters(node_handle);
    _simulated = parameters.simulated();

    //Joint handles
    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) throw std::runtime_error("IntegratedAccelerationController: Error getting effort joint interface from hardware");
    for (size_t i = 0; i < 7; i++)
    {
        _joint_handles.push_back(effort_joint_interface->getHandle(parameters.arm_id() + "_joint" + std::to_string(i + 1)));
    }

    //Random
    Eigen::Matrix<double, 7, 1> joint_position_standard_deviation = parameters.joint_position_standard_deviation();
    Eigen::Matrix<double, 7, 1> joint_velocity_standard_deviation = parameters.joint_velocity_standard_deviation();
    for (size_t i = 0; i < 7; i++)
    {
        _random_position_distributions.push_back(std::normal_distribution<double>(0.0, joint_position_standard_deviation(i)));
        _random_velocity_distributions.push_back(std::normal_distribution<double>(0.0, joint_velocity_standard_deviation(i)));
    }
    _random_engine.seed(time(nullptr));
}

void franka_pole::FrankaState::update(const ros::Time &time)
{
    //Measurement
    _timestamp = time.toSec();
    for (size_t i = 0; i < 7; i++)
    {
        _raw_joint_positions(i) = _joint_handles[i].getPosition();
        _joint_velocities(i) = _joint_handles[i].getVelocity();
    }

    //Adding noise
    if (_simulated)
    {
        for (size_t i = 0; i < 7; i++)
        {
            _joint_positions(i) = _raw_joint_positions(i) + _random_position_distributions[i](_random_engine);
            _joint_velocities(i) = _joint_velocities(i) + _random_velocity_distributions[i](_random_engine);
        }
    }
    else
    {
        _joint_positions = _raw_joint_positions;
    }

    //Basic computations
    _effector_position = _controller->franka_model->effector_forward_kinematics(_joint_positions, &_effector_orientation);
    _effector_velocity = _controller->franka_model->get_effector_jacobian(_joint_positions) * _joint_velocities;

    //Publish
    _controller->publisher->set_franka_timestamp(time);
    _controller->publisher->set_franka_joint_positions(_joint_positions);
    _controller->publisher->set_franka_joint_velocities(_joint_velocities);
    _controller->publisher->set_franka_effector_position(_effector_position);
    _controller->publisher->set_franka_effector_velocity(_effector_velocity.segment<3>(0));
}

double franka_pole::FrankaState::get_timestamp() const
{
    return _timestamp;
}

Eigen::Matrix<double, 7, 1> franka_pole::FrankaState::get_raw_joint_positions() const
{
    return _raw_joint_positions;
}

Eigen::Matrix<double, 7, 1> franka_pole::FrankaState::get_joint_positions() const
{
    return _joint_positions;
}

Eigen::Matrix<double, 7, 1> franka_pole::FrankaState::get_joint_velocities() const
{
    return _joint_velocities;
}

Eigen::Matrix<double, 3, 1> franka_pole::FrankaState::get_effector_position() const
{
    return _effector_position;
}

Eigen::Quaterniond franka_pole::FrankaState::get_effector_orientation() const
{
    return _effector_orientation;
}

Eigen::Matrix<double, 6, 1> franka_pole::FrankaState::get_effector_velocity() const
{
    return _effector_velocity;
}

void franka_pole::FrankaState::set_torque(const Eigen::Matrix<double, 7, 1> &torque)
{
    for (size_t i = 0; i < 7; i++) _joint_handles[i].setCommand(torque(i));
}