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
        if (_controller->is_two_dimensional()) _joint_handles.push_back(position_joint_interface->getHandle(controller->get_arm_id() + "_pole_joint_y")); //Y comes first (if comes)
        _joint_handles.push_back(position_joint_interface->getHandle(controller->get_arm_id() + "_pole_joint_x"));
    }
    catch (const hardware_interface::HardwareInterfaceException &ex)
    {
        ROS_ERROR_STREAM("IntegratedPositionController: Exception getting joint handles: " << ex.what());
        return;
    }

    if (_controller->is_simulated()) _pose_stamped_subscriber = node_handle.subscribe("/vicon/Pole_1/Pole_1", 20, &PoleState::_update, this, ros::TransportHints().reliable().tcpNoDelay());

    _ok = true;
}

bool franka_pole::PoleState::ok() const
{
    return _ok;
}

void franka_pole::PoleState::update(const ros::Time &time)
{
    //Timestamp
    _timestamp = time.toSec();

    //Angles
    _angle(0) = _controller->is_two_dimensional() ? -_joint_handles[1].getPosition() : -_joint_handles[0].getPosition(); //Around X
    _dangle(0) = _controller->is_two_dimensional() ? -_joint_handles[1].getVelocity() : -_joint_handles[0].getVelocity();
    _angle(1) = _controller->is_two_dimensional() ? _joint_handles[0].getPosition() : 0.0; //Around Y
    _dangle(1) = _controller->is_two_dimensional() ? _joint_handles[0].getVelocity() : 0.0;

    Eigen::Matrix<double, 3, 3> rotation = _controller->franka_state->get_effector_orientation().toRotationMatrix();
    rotation = Eigen::AngleAxisd(M_PI, rotation.col(0)) * rotation; //180 around X
    rotation = Eigen::AngleAxisd(_angle(1), rotation.col(1)) * rotation; //Around Y
    rotation = Eigen::AngleAxisd(-_angle(0), rotation.col(0)) * rotation; //Around X
    Eigen::Matrix<double, 3, 1> up = rotation * Eigen::Vector3d::UnitZ();
    _angle(0) = atan2(up(1), up(2));
    _angle(1) = atan2(up(0), up(2));

    //Publish
    _controller->publisher->set_pole_timestamp(time);
    _controller->publisher->set_pole_angle(_angle);
    _controller->publisher->set_pole_dangle(_dangle);
}

double franka_pole::PoleState::get_timestamp() const
{
    return _timestamp;
}

Eigen::Matrix<double, 2, 1> franka_pole::PoleState::get_angle() const
{
    return _angle;
}

Eigen::Matrix<double, 2, 1> franka_pole::PoleState::get_dangle() const
{
    return _dangle;
}