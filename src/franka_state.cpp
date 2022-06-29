#include <pinocchio/fwd.hpp>
#include <franka_pole/franka_state.h>
#include <franka_pole/controller.h>
#include <franka_pole/publisher.h>

franka_pole::FrankaState::FrankaState(Controller *controller, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    _controller = controller;

    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
        ROS_ERROR_STREAM("IntegratedAccelerationController: Error getting state interface from hardware");
        return;
    }
    try
    {
        _state_handle = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(controller->param->arm_id() + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
        ROS_ERROR_STREAM("IntegratedAccelerationController: Exception getting state handle from interface: " << ex.what());
        return;
    }

    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
        ROS_ERROR_STREAM("IntegratedAccelerationController: Error getting effort joint interface from hardware");
        return;
    }
    for (size_t i = 0; i < 7; ++i)
    {
        try
        {
            _joint_handles.push_back(effort_joint_interface->getHandle(controller->param->arm_id() + "_joint" + std::to_string(i + 1)));
        }
        catch (const hardware_interface::HardwareInterfaceException &ex)
        {
            ROS_ERROR_STREAM("IntegratedAccelerationController: Exception getting joint handles: " << ex.what());
            return;
        }
    }

    _ok = true;
}

bool franka_pole::FrankaState::ok() const
{
    return _ok;
}

void franka_pole::FrankaState::update(const ros::Time &time)
{
    //Timestamp
    _timestamp = time.toSec();

    //Joints
    franka::RobotState robot_state = _state_handle->getRobotState();
    _joint_positions = Eigen::Matrix<double, 7, 1>::Map(robot_state.q.data());
    _joint_velocities = Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data());

    //Effector
    _effector_transform = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
    _effector_position = _effector_transform.translation();
    _effector_velocity = (_controller->franka_model->get_effector_jacobian(_joint_positions) * _joint_velocities);
    _effector_orientation = Eigen::Quaterniond(_effector_transform.linear());

    //Publish
    _controller->publisher->set_franka_timestamp(time);
    _controller->publisher->set_franka_effector_position(_effector_position);
    _controller->publisher->set_franka_effector_velocity(_effector_velocity.segment<3>(0));
}

double franka_pole::FrankaState::get_timestamp()
{
    return _timestamp;
}

Eigen::Vector3d franka_pole::FrankaState::get_effector_position()
{
    return _effector_position;
}

Eigen::Matrix<double, 6, 1> franka_pole::FrankaState::get_effector_velocity()
{
    return _effector_velocity;
}

Eigen::Quaterniond franka_pole::FrankaState::get_effector_orientation()
{
    return _effector_orientation;
}

Eigen::Affine3d franka_pole::FrankaState::get_effector_transform()
{
    return _effector_transform;
}

Eigen::Matrix<double, 7, 1> franka_pole::FrankaState::get_joint_positions()
{
    return _joint_positions;
}

Eigen::Matrix<double, 7, 1> franka_pole::FrankaState::get_joint_velocities()
{
    return _joint_velocities;
}

void franka_pole::FrankaState::set_torque(const Eigen::Matrix<double, 7, 1> &torque)
{
    for (size_t i = 0; i < 7; i++) _joint_handles[i].setCommand(torque(i));
}