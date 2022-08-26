#include <franka_pole/franka_model.h>
#include <franka_pole/franka_state.h>
#include <franka_pole/pole_state.h>
#include <franka_pole/parameters.h>
#include <franka_pole/publisher.h>

void franka_pole::PoleState::_callback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
    std::lock_guard<std::mutex> guard(*_mutex);
    Eigen::Quaterniond pole_orientation(msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z);
    _update(pole_orientation, msg->header.stamp);
}

void franka_pole::PoleState::_update(const Eigen::Quaterniond &pole_orientation, ros::Time timestamp)
{
    //Computing absolute angles + noise + filtering
    Eigen::Matrix<double, 3, 1> up = pole_orientation * Eigen::Vector3d::UnitZ();
    Eigen::Matrix<double, 2, 1> angle(-atan2(up(1), up(2)), _parameters->simulated ? atan2(up(0), up(2)) : -atan2(up(0), up(2)));
    angle += _parameters->pole_angle_mean;
    angle += Eigen::Matrix<double, 2, 1>(_random_angle_distributions[0](_random_engine), _random_angle_distributions[1](_random_engine));
    angle = _parameters->pole_angle_filter * _angle + (1.0 - _parameters->pole_angle_filter) * angle;

    //Computing joint angles
    Eigen::Matrix<double, 3, 1> euler = (pole_orientation * _franka_state->get_effector_orientation()).toRotationMatrix().eulerAngles(0,1,2);
    if (euler(0) > M_PI/2) euler(0) -= M_PI;
    else if (euler(0) < -M_PI/2) euler(0) += M_PI;
    if (euler(1) > M_PI/2) euler(1) -= M_PI;
    else if (euler(1) < -M_PI/2) euler(1) += M_PI;
    Eigen::Matrix<double, 2, 1> joint_angle((get_model_freedom(_parameters->model) >= 1) ? euler(0) : 0.0, (get_model_freedom(_parameters->model) == 2) ? euler(1) : 0.0);

    //Differentiation & update
    if (_first)
    {
        _dangle = Eigen::Matrix<double, 2, 1>::Zero();
        _first = false;
    }
    else
    {
        _dangle = _parameters->pole_dangle_filter * _dangle + (1.0 - _parameters->pole_dangle_filter) * ((angle - _angle) / (timestamp.toSec() - _timestamp));
        _joint_dangle = _parameters->pole_dangle_filter * _joint_dangle + (1.0 - _parameters->pole_dangle_filter) * ((joint_angle - _joint_angle) / (timestamp.toSec() - _timestamp));
    }
    _angle = angle;
    _joint_angle = joint_angle;
    _timestamp = timestamp.toSec();

    //Publish
    _publisher->set_pole(
        timestamp,
        _angle,
        _dangle,
        _joint_angle,
        _joint_dangle);
}

franka_pole::PoleState::PoleState(const Parameters *parameters, FrankaModel *franka_model, const FrankaState *franka_state, Publisher *publisher, std::mutex *mutex, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) :
_parameters(parameters), _franka_model(franka_model), _franka_state(franka_state), _publisher(publisher), _mutex(mutex), _timestamp(0.0)
{
    if (_parameters->simulated)
    {
        auto* position_joint_interface = robot_hw->get<hardware_interface::PositionJointInterface>();
        if (get_model_freedom(_parameters->model) >= 1) _joint_handles[0] = position_joint_interface->getHandle(parameters->arm_id + "_pole_joint_x");
        if (get_model_freedom(_parameters->model) == 2) _joint_handles[1] = position_joint_interface->getHandle(parameters->arm_id + "_pole_joint_y");
    }
    else
    {
        if (_parameters->model == Model::D1) _subscriber = node_handle.subscribe("/vicon/Pole_1D/Pole_1D", 10, &PoleState::_callback, this, ros::TransportHints().reliable().tcpNoDelay());
        else if (_parameters->model == Model::D2) _subscriber = node_handle.subscribe("/vicon/Pole_2D/Pole_2D", 10, &PoleState::_callback, this, ros::TransportHints().reliable().tcpNoDelay());
        else if (_parameters->model == Model::D2b) _subscriber = node_handle.subscribe("/vicon/Pole_2Db/Pole_2Db", 10, &PoleState::_callback, this, ros::TransportHints().reliable().tcpNoDelay());
        else if (_parameters->model == Model::D2c) _subscriber = node_handle.subscribe("/vicon/Pole_2Dc/Pole_2Dc", 10, &PoleState::_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    }

    reset(Eigen::Matrix<double, 2, 1>::Zero(), Eigen::Matrix<double, 2, 1>::Zero());
}

void franka_pole::PoleState::reset(const Eigen::Matrix<double, 2, 1> &joint_angle, const Eigen::Matrix<double, 2, 1> &joint_dangle)
{
    //Random
    for (size_t i = 0; i < 2; i++)
    {
        _random_angle_distributions[i] = std::normal_distribution<double>(0.0, _parameters->pole_angle_standard_deviation(i));
    }
    _random_engine.seed(time(nullptr));

    //Initial readings
    _first = true;
    _joint_angle = joint_angle;
    _joint_dangle = joint_dangle;
    _angle = joint_angle;
    _dangle = joint_dangle;
}

void franka_pole::PoleState::update(const ros::Time &time)
{    
    if (_parameters->simulated)
    {
        //Measuring joint angles
        Eigen::Matrix<double, 2, 1> joint_angle(
            (get_model_freedom(_parameters->model) >= 1) ? -_joint_handles[0].getPosition() : 0.0,
            (get_model_freedom(_parameters->model) == 2) ? _joint_handles[1].getPosition() : 0.0
        );

        //Computing absolute angles
        Eigen::Quaterniond pole_orientation;
        _franka_model->pole_forward_kinematics(_franka_state->_exact_joint_positions, joint_angle, &pole_orientation);
        
        _update(pole_orientation, time);
    }
}

double franka_pole::PoleState::get_timestamp()
{
    return _timestamp;
}

Eigen::Matrix<double, 2, 1> franka_pole::PoleState::get_angle()
{
    return _angle;
}

Eigen::Matrix<double, 2, 1> franka_pole::PoleState::get_dangle()
{
    return _dangle;
}

Eigen::Matrix<double, 2, 1> franka_pole::PoleState::get_joint_angle()
{
    return _joint_angle;
}

Eigen::Matrix<double, 2, 1> franka_pole::PoleState::get_joint_dangle()
{
    return _joint_dangle;
}