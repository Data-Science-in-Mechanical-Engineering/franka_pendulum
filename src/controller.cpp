#include <pinocchio/fwd.hpp>
#include <franka_pole/controller.h>
#include <franka_pole/parameters.h>
#include <franka_pole/franka_state.h>
#include <franka_pole/pole_state.h>
#include <franka_pole/publisher.h>
#include <fcntl.h>

void franka_pole::Controller::_command_reset(const franka_pole::CommandReset::ConstPtr &msg)
{
    if (msg->software)
    {
        sem_post(_software_reset_semaphore);
        _software_reset = true;
    }
    else
    {
        _hardware_reset_old_positions = franka_state->get_joint_positions();
        _hardware_reset_counter = 0;
        _hardware_reset = true;
    }
}

bool franka_pole::Controller::_controller_init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    try
    {
        Parameters parameters(node_handle);

        //Creating components
        franka_model = std::make_unique<FrankaModel>(node_handle);
        publisher = std::make_unique<Publisher>(node_handle);
        franka_state = std::make_unique<FrankaState>(this, robot_hw, node_handle);
        pole_state = std::make_unique<PoleState>(this, robot_hw, node_handle);

        //Reading parameters
        _initial_joint_positions = franka_model->effector_inverse_kinematics(parameters.initial_effector_position(), parameters.initial_effector_orientation(), parameters.initial_joint0_position());
        _joint_stiffness = parameters.joint_stiffness();
        _franka_period = parameters.franka_period();
        _pole_period = parameters.pole_period();

        //Opening reset subscribers
        _reset_subscriber = node_handle.subscribe("/franka_pole/command_reset", 10, &franka_pole::Controller::_command_reset, this);
        _software_reset_semaphore = sem_open("/franka_pole_software_reset", O_CREAT, 0644, 0);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("Exception: " << e.what());
        return false;
    }

    return true;
}

void franka_pole::Controller::_controller_starting(const ros::Time &time)
{
}

void franka_pole::Controller::_controller_pre_update(const ros::Time &time, const ros::Duration &period)
{
    if (_franka_period_counter == 0) franka_state->update(time);
    if (_pole_period_counter == 0) pole_state->update(time);
}

void franka_pole::Controller::_controller_post_update(const ros::Time &time, const ros::Duration &period, const Eigen::Matrix<double, 7, 1> &torque)
{
    if (_software_reset)
    {
        int value;
        sem_getvalue(_software_reset_semaphore, &value);
        if (value == 0) _software_reset = false;
        franka_state->set_torque(Eigen::Matrix<double, 7, 1>::Zero());
    }
    else if (_hardware_reset)
    {
        if (_hardware_reset_counter < 5000)
        {
            double factor = (double)_hardware_reset_counter / 5000.0;
            Eigen::Matrix<double, 7, 1> target = factor * _initial_joint_positions + (1.0 - factor) * _hardware_reset_old_positions;
            
            franka_state->set_torque((
                (target - franka_state->get_joint_positions()).array() * _joint_stiffness.array()
                - franka_state->get_joint_velocities().array() * 2 * _joint_stiffness.array().sqrt())
            .matrix());
            _hardware_reset_counter++;
        }
        else _hardware_reset = false;
    }
    else if (is_period())
    {
        _previous_torque = torque;
        franka_state->set_torque(torque);
        publisher->set_command_timestamp(time);
    }
    else
    {
        franka_state->set_torque(_previous_torque);
    }
    publisher->set_reset(_hardware_reset || _software_reset);
    if (is_period()) publisher->publish();
    _franka_period_counter++; if (_franka_period_counter >= _franka_period) _franka_period_counter = 0;
    _pole_period_counter++; if (_pole_period_counter >= _pole_period) _pole_period_counter = 0;
}

bool franka_pole::Controller::is_reset() const
{
    return _hardware_reset || _software_reset;
}

bool franka_pole::Controller::is_period() const
{
    return _franka_period_counter == 0;
}