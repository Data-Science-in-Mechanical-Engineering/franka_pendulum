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
    //Reading parameters
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
        ROS_ERROR_STREAM("franka_pole::Controller: Could not read parameter arm_id");
        return false;
    }

    std::string simulated_str;
    if (!node_handle.getParam("simulated", simulated_str))
    {
        ROS_ERROR_STREAM("franka_pole::Controller: Could not read parameter simulated");
        return false;
    }
    _simulated = simulated_str == "true";

    std::string two_dimensional_str;
    if (!node_handle.getParam("two_dimensional", two_dimensional_str))
    {
        ROS_ERROR_STREAM("franka_pole::Controller: Could not read parameter two_dimensional");
        return false;
    }
    _two_dimensional = two_dimensional_str == "true";

    //Creating components
    franka_model = std::make_unique<FrankaModel>(this, robot_hw, node_handle);
    franka_state = std::make_unique<FrankaState>(this, robot_hw, node_handle);
    pole_state = std::make_unique<PoleState>(this, robot_hw, node_handle);
    publisher = std::make_unique<Publisher>(this, robot_hw, node_handle);
    if (!(franka_state->ok() && pole_state->ok() && publisher->ok())) return false;

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
                (1.0 - (double)_hardware_reset_counter / 5000) * get_initial_joint_positions().segment<7>(0);
            
            franka_state->set_torque((
                (target - franka_state->get_joint_positions()).array() * get_joint_stiffness().array()
                - franka_state->get_joint_velocities().array() * 2 * get_joint_stiffness().array().sqrt())
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

std::string franka_pole::Controller::get_arm_id() const
{
    return _arm_id;
}

bool franka_pole::Controller::is_simulated() const
{
    return _simulated;
}

bool franka_pole::Controller::is_two_dimensional() const
{
    return _two_dimensional;
}

Eigen::Matrix<double, 3, 1> franka_pole::Controller::get_translation_stiffness() const
{
    return Eigen::Matrix<double, 3, 1>::Ones() * 200.0;
}

Eigen::Matrix<double, 3, 1> franka_pole::Controller::get_rotation_stiffness() const
{
    return Eigen::Matrix<double, 3, 1>::Ones() * 100.0;
}

Eigen::Matrix<double, 7, 1> franka_pole::Controller::get_nullspace_stiffness() const
{
    return Eigen::Matrix<double, 7, 1>::Ones() * 10.0;
}

Eigen::Matrix<double, 7, 1> franka_pole::Controller::get_joint_stiffness() const
{
    static const double raw[] =
    {
        600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0
    };
    return Eigen::Matrix<double, 7, 1>::Map(raw);
}

Eigen::Matrix<double, 3, 1> franka_pole::Controller::get_box_center() const
{
    return Eigen::Matrix<double, 3, 1>(0.5, 0.0, 0.5);
}

Eigen::Matrix<double, 3, 1> franka_pole::Controller::get_box_min() const
{
    return Eigen::Matrix<double, 3, 1>(0.25, -0.6, 0.5);
}

Eigen::Matrix<double, 3, 1> franka_pole::Controller::get_box_max() const
{
    return Eigen::Matrix<double, 3, 1>(0.75, 0.6, 0.5);
}

Eigen::Matrix<double, 11, 1> franka_pole::Controller::get_initial_joint_positions() const
{
    static const double raw[] =
    {
        0.0, 0.0, 0.0, -M_PI / 2, 0.0, M_PI / 2, M_PI / 4,
        0.0, 0.0,
        0.0, 0.0
    };
    return Eigen::Matrix<double, 11, 1>::Map(raw);
}

bool franka_pole::Controller::is_reset() const
{
    return _hardware_reset || _software_reset;
}