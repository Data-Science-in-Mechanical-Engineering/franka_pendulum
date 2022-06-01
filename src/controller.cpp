#include <franka_pole/controller.h>
#include <franka_pole/franka_state.h>
#include <franka_pole/pole_state.h>
#include <franka_pole/publisher.h>

std::string franka_pole::Controller::get_arm_id() const
{
    return _arm_id;
}

bool franka_pole::Controller::is_simulated() const
{
    return _simulated;
}

bool franka_pole::Controller::_controller_init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
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

    franka_state = std::make_unique<FrankaState>(this, robot_hw, node_handle);
    pole_state = std::make_unique<PoleState>(this, robot_hw, node_handle);
    publisher = std::make_unique<Publisher>(this, robot_hw, node_handle);
    if (!(franka_state->ok() && pole_state->ok() && publisher->ok())) return false;

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

void franka_pole::Controller::_controller_post_update(const ros::Time &time, const ros::Duration &period)
{
    publisher->publish();
}