#include <franka_pole/franka_model.h>
#include <franka_pole/controller.h>
#include <franka_pole/parameter_reader.h>
#include <franka_pole/parameters.h>
#include <franka_pole/franka_state.h>
#include <franka_pole/pole_state.h>
#include <franka_pole/publisher.h>
#include <fcntl.h>
        
void franka_pole::Controller::_callback(const franka_pole::CommandReset::ConstPtr &msg)
{
    std::lock_guard<std::mutex> guard(_mutex);
    if (msg->software)
    {
        sem_post(_software_reset_semaphore);
        _software_reset = true;
    }
    else
    {
        _hardware_reset_old_positions = franka_state->get_joint_positions();
        _hardware_reset_time = 0.0;
        _hardware_reset = true;
    }
}

void franka_pole::Controller::_reset()
{
    _franka_period_counter = 0;
    _pole_period_counter = 0;
    _command_period_counter = 0;
    _publish_period_counter = 0;
    _software_reset = false;
    _hardware_reset = false;
    _initial_joint_positions = franka_model->effector_inverse_kinematics(parameters->initial_effector_position, parameters->initial_effector_orientation, parameters->initial_joint0_position);
    if (parameters->model == Model::D0) _torque = franka_model->get_gravity9(_initial_joint_positions).segment<7>(0);
    else if (parameters->model == Model::D1) _torque = franka_model->get_gravity10(_initial_joint_positions, parameters->initial_pole_positions).segment<7>(0);
    else _torque = franka_model->get_gravity11(_initial_joint_positions, parameters->initial_pole_positions).segment<7>(0);
    _init_level1(_robot_hw, _node_handle);
}

bool franka_pole::Controller::_init_level0(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    std::lock_guard<std::mutex> guard(_mutex);
    _robot_hw = robot_hw;
    _node_handle = node_handle;

    try
    {
        ros::TransportHints().tcpNoDelay();

        //Creating components
        ParameterReader parameter_reader(node_handle);
        parameters = new Parameters(parameter_reader, node_handle);
        franka_model = new FrankaModel(parameters);
        publisher = new Publisher(parameters, node_handle);
        franka_state = new FrankaState(parameters, franka_model, publisher, robot_hw);
        if (parameters->model != Model::D0) pole_state = new PoleState(parameters, franka_model, franka_state, publisher, robot_hw, node_handle);    

        //Opening reset subscribers
        _reset_subscriber = node_handle.subscribe("/franka_pole/command_reset", 10, &franka_pole::Controller::_callback, this, ros::TransportHints().reliable().tcpNoDelay());
        _software_reset_semaphore = sem_open("/franka_pole_software_reset", O_CREAT, 0644, 0);
        _software_reset = true;
        _hardware_reset = false;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("Exception: " << e.what());
        return false;
    }
    return true;
}

void franka_pole::Controller::_starting_level0(const ros::Time &time)
{
    std::lock_guard<std::mutex> guard(_mutex);
}

void franka_pole::Controller::_update_level0(const ros::Time &time, const ros::Duration &period)
{
    std::lock_guard<std::mutex> guard(_mutex);
    if (_software_reset)
    {
        int value;
        sem_getvalue(_software_reset_semaphore, &value);
        if (value == 0) _reset();
    }
    else if (_hardware_reset)
    {
        if (_hardware_reset_time < parameters->hardware_reset_duration)
        {
            double factor = _hardware_reset_time / parameters->hardware_reset_duration;
            Eigen::Matrix<double, 7, 1> target_joint_positions = factor * _initial_joint_positions + (1.0 - factor) * _hardware_reset_old_positions;
            
            _torque = (
                (target_joint_positions - franka_state->get_joint_positions()).array() * parameters->hardware_reset_stiffness.array() +
                ( - franka_state->get_joint_velocities()).array() * parameters->hardware_reset_damping.array()
            ).matrix();
            _hardware_reset_time += 0.001;
        }
        else _reset();
    }
    
    if (++_franka_period_counter >= parameters->franka_period) { franka_state->update(time); _franka_period_counter = 0; }
    if (pole_state != nullptr && ++_pole_period_counter >= parameters->pole_period) { pole_state->update(time); _pole_period_counter = 0; }
    if (++_command_period_counter >= parameters->command_period) { if (!_software_reset && !_hardware_reset) _torque = _get_torque_level1(time, period); _command_period_counter = 0; }
    if (++_publish_period_counter >= parameters->publish_period) { publisher->publish(); _publish_period_counter = 0; }

    franka_state->set_torque(_torque);
}

franka_pole::Controller::~Controller()
{
    if (parameters != nullptr) delete parameters;
    if (franka_model != nullptr) delete franka_model;
    if (franka_state != nullptr) delete franka_state;
    if (pole_state != nullptr) delete pole_state;
    if (publisher != nullptr) delete publisher;
}