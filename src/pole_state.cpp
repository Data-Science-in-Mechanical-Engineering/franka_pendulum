#include <franka_pole/franka_model.h>
#include <franka_pole/franka_state.h>
#include <franka_pole/pole_state.h>
#include <franka_pole/parameters.h>
#include <franka_pole/publisher.h>

void franka_pole::PoleState::_callback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
    std::lock_guard<std::mutex> guard(*_mutex);

    //Measuring absolute angles + filtering
    Eigen::Quaterniond pole_orientation(msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z);
    Eigen::Matrix<double, 3, 1> up = pole_orientation * Eigen::Vector3d::UnitZ();
    Eigen::Matrix<double, 2, 1> angle(-atan2(up(1), up(2)), -atan2(up(0), up(2)));
    angle = _parameters->pole_angle_filter * _angle + (1.0 - _parameters->pole_angle_filter) * angle;

    //Getting joint angles
    Eigen::Matrix<double, 3, 1> euler = (pole_orientation * _franka_state->get_effector_orientation()).toRotationMatrix().eulerAngles(0,1,2);
    if (euler(0) > M_PI/2) euler(0) -= M_PI;
    else if (euler(0) < -M_PI/2) euler(0) += M_PI;
    if (euler(1) > M_PI/2) euler(1) -= M_PI;
    else if (euler(1) < -M_PI/2) euler(1) += M_PI;
    Eigen::Matrix<double, 2, 1> joint_angle(euler(0), (_parameters->model == Model::D2 || _parameters->model == Model::D2b) ? euler(1) : 0.0);

    //Differentiation & update
    if (_first)
    {
        _dangle = Eigen::Matrix<double, 2, 1>::Zero();
        _first = false;
    }
    else
    {
        _dangle = _parameters->pole_dangle_filter * _dangle + (1.0 - _parameters->pole_dangle_filter) * ((angle - _angle) / (msg->header.stamp.toSec() - _timestamp));
        _joint_dangle = _parameters->pole_dangle_filter * _joint_dangle + (1.0 - _parameters->pole_dangle_filter) * ((joint_angle - _joint_angle) / (msg->header.stamp.toSec() - _timestamp));
    }
    _angle = angle;
    _joint_angle = joint_angle;
    _timestamp = msg->header.stamp.toSec();

    //Publish
    _publisher->set_pole(
        msg->header.stamp,
        _angle,
        _dangle,
        _joint_angle,
        _joint_dangle);
}

franka_pole::PoleState::PoleState(const Parameters *parameters, FrankaModel *franka_model, const FrankaState *franka_state, Publisher *publisher, std::mutex *mutex, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) :
_parameters(parameters), _franka_model(franka_model), _franka_state(franka_state), _publisher(publisher), _mutex(mutex)
{
    if (_parameters->simulated)
    {
        auto* position_joint_interface = robot_hw->get<hardware_interface::PositionJointInterface>();
        if (_parameters->model == Model::D1 || _parameters->model == Model::D2 || _parameters->model == Model::D2b) _joint_handles[0] = position_joint_interface->getHandle(parameters->arm_id + "_pole_joint_x");
        if (_parameters->model == Model::D2 || _parameters->model == Model::D2b) _joint_handles[1] = position_joint_interface->getHandle(parameters->arm_id + "_pole_joint_y");
    }
    else
    {
        if (_parameters->model == Model::D1) _subscriber = node_handle.subscribe("/vicon/Pole_1D/Pole_1D", 10, &PoleState::_callback, this, ros::TransportHints().reliable().tcpNoDelay());
        if (_parameters->model == Model::D2) _subscriber = node_handle.subscribe("/vicon/Pole_2D/Pole_2D", 10, &PoleState::_callback, this, ros::TransportHints().reliable().tcpNoDelay());
        if (_parameters->model == Model::D2b) _subscriber = node_handle.subscribe("/vicon/Pole_2Db/Pole_2Db", 10, &PoleState::_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    }

    //Random
    Eigen::Matrix<double, 2, 1> pole_angle_standard_deviation = _parameters->pole_angle_standard_deviation;
    for (size_t i = 0; i < 2; i++)
    {
        _random_angle_distributions[i] = std::normal_distribution<double>(0.0, pole_angle_standard_deviation(i));
    }
    _random_engine.seed(time(nullptr));

    //Set zeros
    reset(Eigen::Matrix<double, 2, 1>::Zero(), Eigen::Matrix<double, 2, 1>::Zero());
}

void franka_pole::PoleState::reset(const Eigen::Matrix<double, 2, 1> &joint_angle, const Eigen::Matrix<double, 2, 1> &joint_dangle)
{
    bool _first = true;
    _joint_angle = joint_angle;
    _joint_dangle = joint_dangle;
    _angle = joint_angle;
    _dangle = joint_dangle;
}

void franka_pole::PoleState::update(const ros::Time &time)
{    
    if (_parameters->simulated)
    {
        //Measuring joint angles + adding noise + filtering
        Eigen::Matrix<double, 2, 1> joint_angle(
            (_parameters->model == Model::D1 || _parameters->model == Model::D2 || _parameters->model == Model::D2b) ? -_joint_handles[0].getPosition() + _random_angle_distributions[0](_random_engine) : 0.0,
            (_parameters->model == Model::D2 || _parameters->model == Model::D2b) ? _joint_handles[1].getPosition() + _random_angle_distributions[1](_random_engine) : 0.0
        );
        joint_angle = _parameters->pole_angle_filter * _joint_angle + (1.0 - _parameters->pole_angle_filter) * joint_angle;

        //Getting absolute angles
        Eigen::Quaterniond pole_orientation;
        _franka_model->pole_forward_kinematics(_franka_state->_exact_joint_positions, joint_angle, &pole_orientation);
        Eigen::Matrix<double, 3, 1> up = pole_orientation * Eigen::Vector3d::UnitZ();
        Eigen::Matrix<double, 2, 1> angle(-atan2(up(1), up(2)), atan2(up(0), up(2)));

        //Differentiation + filtering + update
        if (_first)
        {
            _joint_dangle = Eigen::Matrix<double, 2, 1>::Zero();
            _dangle = Eigen::Matrix<double, 2, 1>::Zero();
            _first = false;
        }
        else
        {
            _joint_dangle = _parameters->pole_dangle_filter * _joint_dangle + (1.0 - _parameters->pole_dangle_filter) * ((joint_angle - _joint_angle) / (time.toSec() - _timestamp));
            _dangle = _parameters->pole_dangle_filter * _dangle + (1.0 - _parameters->pole_dangle_filter) * ((angle - _angle) / (time.toSec() - _timestamp));
        }
        _angle = angle;
        _joint_angle = joint_angle;
        _timestamp = time.toSec();

        /*
        //Exact joint velocity
        _joint_dangle(0) = (_parameters->model == Model::D1 || _parameters->model == Model::D2 || _parameters->model == Model::D2b) ? -_joint_handles[0].getVelocity() : 0.0;
        _joint_dangle(1) = (_parameters->model == Model::D2 || _parameters->model == Model::D2b) ? _joint_handles[1].getVelocity() : 0.0;
        */

        //Publish
        _publisher->set_pole(
            time,
            _angle,
            _dangle,
            _joint_angle,
            _joint_dangle);
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