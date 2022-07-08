#include <pinocchio/fwd.hpp>
#include <franka_pole/pole_state.h>
#include <franka_pole/controller.h>
#include <franka_pole/parameters.h>
#include <franka_pole/publisher.h>

void franka_pole::PoleState::_update(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    _timestamp = msg->header.stamp.toSec();
    //todo
}

franka_pole::PoleState::PoleState(Controller *controller, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) : _controller(controller)
{
    Parameters parameters(node_handle);
    _two_dimensional = parameters.two_dimensional();

    //Joints
    auto* position_joint_interface = robot_hw->get<hardware_interface::PositionJointInterface>();
    if (_two_dimensional) _joint_handles.push_back(position_joint_interface->getHandle(parameters.arm_id() + "_pole_joint_y")); //Y comes first (if comes)
    _joint_handles.push_back(position_joint_interface->getHandle(parameters.arm_id() + "_pole_joint_x"));

    if (parameters.simulated()) _pose_stamped_subscriber = node_handle.subscribe("/vicon/Pole_1/Pole_1", 20, &PoleState::_update, this, ros::TransportHints().reliable().tcpNoDelay());

    //Random
    Eigen::Matrix<double, 2, 1> pole_angle_standard_deviation = parameters.pole_angle_standard_deviation();
    for (size_t i = 0; i < 2; i++)
    {
        _random_angle_distributions.push_back(std::normal_distribution<double>(0.0, pole_angle_standard_deviation(i)));
    }
    _random_engine.seed(time(nullptr));
}
size_t COUNTER = 0;

void franka_pole::PoleState::update(const ros::Time &time)
{
    if (COUNTER == 0)
    {
    //Timestamp
    _timestamp = time.toSec();

    //Joint angles
    _joint_angle(0) = _two_dimensional ? -_joint_handles[1].getPosition() : -_joint_handles[0].getPosition(); //Around X
    _joint_angle(1) = _two_dimensional ? _joint_handles[0].getPosition() : 0.0; //Around Y
    
    //Absolute angles
    Eigen::Quaterniond pole_orientation;
    Eigen::Matrix<double, 3, 1> pole_position = _controller->franka_model->pole_forward_kinematics(_controller->franka_state->get_raw_joint_positions(), _joint_angle, &pole_orientation);
    Eigen::Matrix<double, 3, 3> rotation = _controller->franka_state->get_effector_orientation().toRotationMatrix();
    Eigen::Matrix<double, 3, 1> up = pole_orientation.inverse() /*Not a bug, really inverse (for some reason)*/ * Eigen::Vector3d::UnitZ();
    _angle(0) = atan2(up(1), up(2));
    _angle(1) = atan2(up(0), up(2));

    //Noise
    _joint_angle(0) += _random_angle_distributions[0](_random_engine);
    _angle(0) += _random_angle_distributions[0](_random_engine);
    if (_two_dimensional)
    {
        _joint_angle(1) += _random_angle_distributions[1](_random_engine);
        _angle(1) += _random_angle_distributions[1](_random_engine);
    }

    //Velocity (too precise, need to add noise/numeric differentiation)
    _joint_dangle(0) = _two_dimensional ? -_joint_handles[1].getVelocity() : -_joint_handles[0].getVelocity();
    _joint_dangle(1) = _two_dimensional ? _joint_handles[0].getVelocity() : 0.0;

    //Publish
    _controller->publisher->set_pole_timestamp(time);
    _controller->publisher->set_pole_joint_angle(_joint_angle);
    _controller->publisher->set_pole_joint_dangle(_joint_dangle);
    _controller->publisher->set_pole_angle(_angle);
    }
    COUNTER = (COUNTER + 1) % 10;
}

double franka_pole::PoleState::get_timestamp() const
{
    return _timestamp;
}

Eigen::Matrix<double, 2, 1> franka_pole::PoleState::get_joint_angle() const
{
    return _joint_angle;
}

Eigen::Matrix<double, 2, 1> franka_pole::PoleState::get_joint_dangle() const
{
    return _joint_dangle;
}

Eigen::Matrix<double, 2, 1> franka_pole::PoleState::get_angle() const
{
    return _angle;
}