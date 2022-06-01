#include <franka_pole/pole_state.h>
#include <franka_pole/controller.h>
#include <franka_pole/publisher.h>

void franka_pole::PoleState::_update(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    _timestamp = msg->header.stamp.toSec();
    //todo
}

franka_pole::PoleState::PoleState(Controller *controller, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) : _controller(controller)
{
    auto* position_joint_interface = robot_hw->get<hardware_interface::PositionJointInterface>();
    try
    {
        _joint_handles.push_back(position_joint_interface->getHandle("panda_lower_upper"));
    }
    catch (const hardware_interface::HardwareInterfaceException &ex)
    {
        ROS_ERROR_STREAM("IntegratedPositionController: Exception getting joint handles: " << ex.what());
        return;
    }

    if (_controller->is_simulated()) _pose_stamped_subscriber = node_handle.subscribe("/vicon/Pole_1/Pole_1", 20, &PoleState::_update, this, ros::TransportHints().reliable().tcpNoDelay());

    _ok = true;
}

bool franka_pole::PoleState::ok()
{
    return _ok;
}

void franka_pole::PoleState::update(const ros::Time &time)
{
    //Timestamp
    _timestamp = time.toSec();

    //Angles
    _angle = _joint_handles[0].getPosition();
    _dangle = _joint_handles[0].getVelocity();

    //Publish
    _controller->publisher->set_pole_timestamp(time);
    _controller->publisher->set_pole_angle(_angle);
    _controller->publisher->set_pole_dangle(_dangle);
}

double franka_pole::PoleState::get_timestamp()
{
    return _timestamp;
}

double franka_pole::PoleState::get_angle()
{
    return _angle;
}

double franka_pole::PoleState::get_dangle()
{
    return _dangle;
}