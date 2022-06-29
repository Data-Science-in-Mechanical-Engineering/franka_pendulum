#include <pinocchio/fwd.hpp>
#include <franka_pole/controller.h>
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
        _hardware_reset_initial = franka_state->get_joint_positions();
        _hardware_reset_counter = 0;
        _hardware_reset = true;
    }
}

bool franka_pole::Controller::_controller_init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    //Creating components
    try
    {
        param = std::make_unique<Parameters>(this, robot_hw, node_handle);
        franka_model = std::make_unique<FrankaModel>(this, robot_hw, node_handle);
        franka_state = std::make_unique<FrankaState>(this, robot_hw, node_handle);
        pole_state = std::make_unique<PoleState>(this, robot_hw, node_handle);
        publisher = std::make_unique<Publisher>(this, robot_hw, node_handle);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("Exception: " << e.what());
    }

    //Opening reset subscribers
    _reset_subscriber = node_handle.subscribe("/franka_pole/command_reset", 10, &franka_pole::Controller::_command_reset, this);
    _software_reset_semaphore = sem_open("/franka_pole_software_reset", O_CREAT, 0644, 0);

    return true;
}

void franka_pole::Controller::_controller_starting(const ros::Time &time)
{
}

void franka_pole::Controller::_controller_pre_update(const ros::Time &time, const ros::Duration &period)
{
    franka_state->update(time);
    pole_state->update(time);
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
            Eigen::Matrix<double, 7, 1> target =
                ((double)_hardware_reset_counter / 5000) * _hardware_reset_initial +
                (1.0 - (double)_hardware_reset_counter / 5000) * param->initial_joint_positions().segment<7>(0);
            
            franka_state->set_torque((
                (target - franka_state->get_joint_positions()).array() * param->joint_stiffness().array()
                - franka_state->get_joint_velocities().array() * 2 * param->joint_stiffness().array().sqrt())
            .matrix());
            _hardware_reset_counter++;
        }
        else _hardware_reset = false;
    }
    else
    {
        franka_state->set_torque(torque);
    }
    publisher->set_reset(_hardware_reset || _software_reset);
    publisher->publish();
}

bool franka_pole::Controller::is_reset() const
{
    return _hardware_reset || _software_reset;
}