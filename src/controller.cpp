#include <franka_pole/franka_model.h>
#include <franka_pole/controller.h>
#include <franka_pole/parameter_reader.h>
#include <franka_pole/parameters.h>
#include <franka_pole/franka_state.h>
#include <franka_pole/pole_state.h>
#include <franka_pole/publisher.h>

#include <franka_msgs/ErrorRecoveryActionGoal.h>
#include <fcntl.h>

void franka_pole::Controller::_initiate_hardware_reset()
{
    if (!parameters->simulated) _hardware_reset_publisher.publish(franka_msgs::ErrorRecoveryActionGoal());
    _hardware_reset_start_position = franka_state->get_effector_position();
    _hardware_reset_start_orientation = franka_state->get_effector_orientation();
    _hardware_reset_start_positions = franka_state->get_joint_positions();
    const double hint[] = { 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4 };
    _hardware_reset_end_positions = franka_model->effector_inverse_kinematics(parameters->initial_effector_position, parameters->initial_effector_orientation, parameters->initial_joint0_position, Eigen::Matrix<double, 7, 1>::Map(hint));
    _hardware_reset_time = 0.0;
    _reset_mode = ResetMode::hardware_reset;
}

void franka_pole::Controller::_initiate_software_reset()
{
    sem_post(_software_reset_semaphore);
    _reset_mode = ResetMode::software_reset;
}

void franka_pole::Controller::_initiate_normal_mode()
{
    if (pole_state != nullptr) pole_state->reset(parameters->initial_pole_positions, parameters->initial_pole_velocities);
    franka_state->reset();
    _init_level1(_robot_hw, _node_handle);
    _reset_mode = ResetMode::normal;
}

void franka_pole::Controller::_callback(const franka_pole::CommandReset::ConstPtr &msg)
{
    std::lock_guard<std::mutex> guard(mutex);
    if (msg->software) _initiate_software_reset();
    else _initiate_hardware_reset();
}

bool franka_pole::Controller::_init_level0(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    std::lock_guard<std::mutex> guard(mutex);
    
    try
    {
        //Technical
        _robot_hw = robot_hw;
        _node_handle = node_handle;
        ros::TransportHints().tcpNoDelay();

        //Time
        _franka_period_counter = 0;
        _pole_period_counter = 0;
        _command_period_counter = 0;
        _publish_period_counter = 0;

        //Creating components
        ParameterReader parameter_reader(node_handle);
        parameters = new Parameters(&mutex, parameter_reader, node_handle, true);
        franka_model = new FrankaModel(parameters);
        publisher = new Publisher(parameters, node_handle);
        franka_state = new FrankaState(parameters, franka_model, publisher, robot_hw);
        franka_state->reset();
        if (parameters->model != Model::D0)
        {
            pole_state = new PoleState(parameters, franka_model, franka_state, publisher, &mutex, robot_hw, node_handle);
            pole_state->reset(parameters->initial_pole_positions, parameters->initial_pole_velocities);
        }

        //Reset
        _reset_subscriber = node_handle.subscribe("/" + parameters->namespacee + "/command_reset", 10, &franka_pole::Controller::_callback, this, ros::TransportHints().reliable().tcpNoDelay());
        if (parameters->simulated)
        {
            _software_reset_semaphore = sem_open(("/" + parameters->namespacee + "_" + parameters->arm_id + "_software_reset").c_str(), O_CREAT, 0644, 0);
            if (_software_reset_semaphore == SEM_FAILED) throw std::runtime_error("franka_pole::Controller::_init_level0(): sem_open failed");
            _initiate_software_reset();
        }
        else
        {
            _hardware_reset_publisher = node_handle.advertise<franka_msgs::ErrorRecoveryActionGoal>("/franka_control/error_recovery/goal", 10);
            _initiate_hardware_reset();
        }

        //Intial output
        #ifdef FRANKA_POLE_VELOCITY_INTERFACE
            _velocity = Eigen::Matrix<double, 6, 1>::Zero();
        #else
            _torque = Eigen::Matrix<double, 7, 1>::Zero();
        #endif
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
    std::lock_guard<std::mutex> guard(mutex);
}

void franka_pole::Controller::_update_level0(const ros::Time &time, const ros::Duration &period)
{
    //Reading data
    if (_franka_period_counter == 0) franka_state->update(time);
    if (_pole_period_counter == 0 && pole_state != nullptr) pole_state->update(time);

    //Software reset
    if (_reset_mode == ResetMode::software_reset)
    {
        //Do nothing
        #ifdef FRANKA_POLE_VELOCITY_INTERFACE
            _velocity = Eigen::Matrix<double, 6, 1>::Zero();
        #else
            _torque = Eigen::Matrix<double, 7, 1>::Zero();
        #endif

        //Wait for semaphore
        int value;
        sem_getvalue(_software_reset_semaphore, &value);
        if (value == 0) _initiate_normal_mode();
    }
    //Hardware reset
    else if (_reset_mode == ResetMode::hardware_reset)
    {
        const double hardware_recovery_duration = 5.0;
        const double hardware_reset_duration = std::max(hardware_recovery_duration, _hardware_reset_time);
        if (_hardware_reset_time < hardware_reset_duration)
        {
            //Do nothing
            #ifdef FRANKA_POLE_VELOCITY_INTERFACE
                _velocity = Eigen::Matrix<double, 6, 1>::Zero();
            #else
                _torque = (
                    (_hardware_reset_start_positions - franka_state->get_joint_positions()).array() * parameters->hardware_reset_stiffness.array() +
                    ( - franka_state->get_joint_velocities()).array() * parameters->hardware_reset_damping.array()
                ).matrix();
            #endif
            _hardware_reset_time += period.toSec();
        }
        else if (_hardware_reset_time < parameters->hardware_reset_duration)
        {
            //Move to start
            #ifdef FRANKA_POLE_VELOCITY_INTERFACE
                _velocity.segment<3>(0) = (parameters->initial_effector_position - _hardware_reset_old_position) / (parameters->hardware_reset_duration - 4.0);
                _velocity.segment<3>(3) = Eigen::Matrix<double, 3, 1>::Zero();
            #else
                double factor = (_hardware_reset_time - hardware_reset_duration) / (parameters->hardware_reset_duration - hardware_reset_duration);
                Eigen::Matrix<double, 7, 1> target_joint_positions = factor * _hardware_reset_end_positions + (1.0 - factor) * _hardware_reset_start_positions;
                _torque = (
                    (target_joint_positions - franka_state->get_joint_positions()).array() * parameters->hardware_reset_stiffness.array() +
                    ( - franka_state->get_joint_velocities()).array() * parameters->hardware_reset_damping.array()
                ).matrix();
            #endif
            _hardware_reset_time += period.toSec();
        }
        else _initiate_normal_mode();
    }
    //Normal operation
    else
    {
        #ifdef FRANKA_POLE_VELOCITY_INTERFACE
            if (_command_period_counter == 0) _velocity = _get_velocity_level1(time, ros::Duration(0,1000000*parameters->command_period));
        #else
            if (_command_period_counter == 0) _torque = _get_torque_level1(time, ros::Duration(0,1000000*parameters->command_period));
        #endif
    }
    
    //Output
    if (_publish_period_counter == 0) publisher->publish();
    #ifdef FRANKA_POLE_VELOCITY_INTERFACE
        franka_state->_set_velocity(_velocity);
    #else
        franka_state->_set_torque(_torque);
    #endif

    //Updating time
    if (++_franka_period_counter >= parameters->franka_period) _franka_period_counter = 0;
    if (++_pole_period_counter >= parameters->pole_period) _pole_period_counter = 0;
    if (++_command_period_counter >= parameters->command_period) _command_period_counter = 0;
    if (++_publish_period_counter >= parameters->publish_period) _publish_period_counter = 0;
}

franka_pole::Controller::~Controller()
{
    if (parameters != nullptr) delete parameters;
    if (franka_model != nullptr) delete franka_model;
    if (franka_state != nullptr) delete franka_state;
    if (pole_state != nullptr) delete pole_state;
    if (publisher != nullptr) delete publisher;
}