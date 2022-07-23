#include <pinocchio/fwd.hpp>
#include <franka_pole/pole_state.h>
#include <franka_pole/controller.h>
#include <franka_pole/parameters.h>
#include <franka_pole/publisher.h>

void franka_pole::PoleState::_update(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
    //Absolute angles
    Eigen::Matrix<double, 3, 1> up = Eigen::Quaterniond(msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z) * Eigen::Vector3d::UnitZ();
    Eigen::Matrix<double, 2, 1> angle(-atan2(up(1), up(2)), -atan2(up(0), up(2)));

    //Differentiation & update
    if (_angle.hasNaN()) _dangle = Eigen::Matrix<double, 2, 1>::Zero();
    else _dangle = (angle - _angle) / (msg->header.stamp.toSec() - _timestamp);
    _angle = angle;
    _timestamp = msg->header.stamp.toSec();

    //Publish
    _controller->publisher->set_pole_timestamp(msg->header.stamp);
    _controller->publisher->set_pole_angle(_angle);
    _controller->publisher->set_pole_dangle(_dangle);
}

franka_pole::PoleState::PoleState(Controller *controller, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) : _controller(controller)
{
    Parameters parameters(node_handle);
    _model = parameters.model();
    _simulated = parameters.simulated();

    if (_simulated)
    {
        auto* position_joint_interface = robot_hw->get<hardware_interface::PositionJointInterface>();
        if (_model == Model::D2 || _model == Model::D2b) _joint_handles.push_back(position_joint_interface->getHandle(parameters.arm_id() + "_pole_joint_y")); //Y comes first (if comes)
        _joint_handles.push_back(position_joint_interface->getHandle(parameters.arm_id() + "_pole_joint_x"));
    }
    else
    {
        const char *name = (_model == Model::D1) ? "/vicon/Pole_1/Pole_1" : "/vicon/Pole_2D/Pole_2D";
        _pose_stamped_subscriber = node_handle.subscribe(name, 20, &PoleState::_update, this, ros::TransportHints().reliable().tcpNoDelay());
    }

    //Random
    Eigen::Matrix<double, 2, 1> pole_angle_standard_deviation = parameters.pole_angle_standard_deviation();
    for (size_t i = 0; i < 2; i++)
    {
        _random_angle_distributions.push_back(std::normal_distribution<double>(0.0, pole_angle_standard_deviation(i)));
    }
    _random_engine.seed(time(nullptr));
}

void franka_pole::PoleState::update(const ros::Time &time)
{
    if (_simulated)
    {
        //Measuring joint angles
        Eigen::Matrix<double, 2, 1> joint_angle;
        joint_angle(0) = (_model == Model::D2 || _model == Model::D2b) ? -_joint_handles[1].getPosition() : -_joint_handles[0].getPosition(); //Around X
        joint_angle(1) = (_model == Model::D2 || _model == Model::D2b) ? _joint_handles[0].getPosition() : 0.0; //Around Y
        
        //Adding noise
        joint_angle(0) += _random_angle_distributions[0](_random_engine);
        if (_model == Model::D2 || _model == Model::D2b) joint_angle(1) += _random_angle_distributions[1](_random_engine);

        //Getting absolute angles
        Eigen::Quaterniond pole_orientation;
        Eigen::Matrix<double, 3, 1> pole_position = _controller->franka_model->pole_forward_kinematics(_controller->franka_state->get_raw_joint_positions(), joint_angle, &pole_orientation);
        Eigen::Matrix<double, 3, 3> rotation = _controller->franka_state->get_effector_orientation().toRotationMatrix();
        Eigen::Matrix<double, 3, 1> up = pole_orientation * Eigen::Vector3d::UnitZ();
        Eigen::Matrix<double, 2, 1> angle(-atan2(up(1), up(2)), atan2(up(0), up(2)));

        //Differentiation & update
        if (_joint_angle.hasNaN()) _joint_dangle = Eigen::Matrix<double, 2, 1>::Zero();
        else _joint_dangle = (joint_angle - _joint_angle) / (time.toSec() - _timestamp);
        if (_angle.hasNaN()) _dangle = Eigen::Matrix<double, 2, 1>::Zero();
        else _dangle = (angle - _angle) / (time.toSec() - _timestamp);
        _angle = angle;
        _joint_angle = joint_angle;
        _timestamp = time.toSec();

        /*
        //Exact joint velocity
        _joint_dangle(0) = (_model == Model::D2 || _model == Model::D2b) ? -_joint_handles[1].getVelocity() : -_joint_handles[0].getVelocity();
        _joint_dangle(1) = (_model == Model::D2 || _model == Model::D2b) ? _joint_handles[0].getVelocity() : 0.0;
        */

        //Publish
        _controller->publisher->set_pole_timestamp(time);
        _controller->publisher->set_pole_angle(_angle);
        _controller->publisher->set_pole_dangle(_dangle);
        _controller->publisher->set_pole_joint_angle(_joint_angle);
        _controller->publisher->set_pole_joint_dangle(_joint_dangle);
    }
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

Eigen::Matrix<double, 2, 1> franka_pole::PoleState::get_joint_angle() const
{
    return _joint_angle;
}

Eigen::Matrix<double, 2, 1> franka_pole::PoleState::get_joint_dangle() const
{
    return _joint_dangle;
}