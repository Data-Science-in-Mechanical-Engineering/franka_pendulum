#include <franka_pole/franka_model.h>
#include <franka_pole/pseudo_inverse.h>
#include <franka_pole/controller.h>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <ros/package.h>

franka_pole::FrankaModel::FrankaModel(Controller *controller, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    _controller = controller;

    //ROS Model
    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) throw std::runtime_error("FrankaModel: Error getting model interface from hardware");
    _model_handle = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(controller->param->arm_id() + "_model"));
    
    //Pinocchio model
    std::string package_path = ros::package::getPath("franka_pole");
    pinocchio::urdf::buildModel(package_path + (controller->param->two_dimensional() ? "/robots/franka_pole_2D.urdf" : "/robots/franka_pole.urdf"), _pinocchio_model);
    _pinocchio_data = pinocchio::Data(_pinocchio_model);
    for (size_t i = 0; i < 7; i++)
    {
        std::string name = controller->param->arm_id() + "_joint" + std::to_string(i + 1);
        if (!_pinocchio_model.existJointName(name)) throw std::runtime_error("FrankaModel: Joint " + controller->param->arm_id() + "_joint" + std::to_string(i + 1) + " not found");
        _pinocchio_joint_ids[i] = _pinocchio_model.getJointId(name);
    }
    for (size_t i = 0; i < 2; i++)
    {
        std::string name = controller->param->arm_id() + "_finger_joint" + std::to_string(i + 1);
        if (!_pinocchio_model.existJointName(name)) throw std::runtime_error("FrankaModel: Joint " + controller->param->arm_id() + "_finger_joint" + std::to_string(i + 1) + " not found");
        _pinocchio_joint_ids[i+7] = _pinocchio_model.getJointId(name);
    }
    if (controller->param->two_dimensional())
    {
        if (!_pinocchio_model.existJointName(controller->param->arm_id() + "_pole_joint_y")) throw std::runtime_error("FrankaModel: Joint " + controller->param->arm_id() + "_pole_joint_y not found");
        _pinocchio_joint_ids[9] = _pinocchio_model.getJointId(controller->param->arm_id() + "_pole_joint_y");
        if (!_pinocchio_model.existJointName(controller->param->arm_id() + "_pole_joint_x")) throw std::runtime_error("FrankaModel: Joint " + controller->param->arm_id() + "_pole_joint_x not found");
        _pinocchio_joint_ids[10] = _pinocchio_model.getJointId(controller->param->arm_id() + "_pole_joint_x");
    }
    else
    {
        if (!_pinocchio_model.existJointName(controller->param->arm_id() + "_pole_joint_x")) throw std::runtime_error("FrankaModel: Joint " + controller->param->arm_id() + "_pole_joint_x not found");
        _pinocchio_joint_ids[9] = _pinocchio_model.getJointId(controller->param->arm_id() + "_pole_joint_x");
    }
    _pinocchio_model.gravity = pinocchio::Motion::Zero();
    
    _ok = true;
}

bool franka_pole::FrankaModel::ok() const
{
    return _ok;
}

Eigen::Matrix<double, 7, 1> franka_pole::FrankaModel::get_gravity(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 2, 1> &pole_positions)
{
    if (_controller->param->two_dimensional())
    {
        Eigen::Matrix<double, 11, 1> q = Eigen::Matrix<double, 11, 1>::Zero();
        q.segment<7>(0) = joint_positions;
        q.segment<2>(9) = pole_positions; //Not right, need to fix
        return pinocchio::computeGeneralizedGravity(_pinocchio_model, _pinocchio_data, q);
    }
    else
    {
        Eigen::Matrix<double, 10, 1> q = Eigen::Matrix<double, 10, 1>::Zero();
        q.segment<7>(0) = joint_positions;
        q(9) = pole_positions(0);
        return pinocchio::computeGeneralizedGravity(_pinocchio_model, _pinocchio_data, q);
    }
}

Eigen::Matrix<double, 7, 1> franka_pole::FrankaModel::get_coriolis(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 7, 1> &joint_velocities, const Eigen::Matrix<double, 2, 1> &pole_positions, const Eigen::Matrix<double, 2, 1> &pole_velocities)
{
    if (_controller->param->two_dimensional())
    {
        Eigen::Matrix<double, 11, 1> q11 = Eigen::Matrix<double, 11, 1>::Zero();
        q11.segment<7>(0) = joint_positions;
        q11(9) = pole_positions(1);
        q11(10) = pole_positions(0);
        Eigen::Matrix<double, 11, 1> v11 = Eigen::Matrix<double, 11, 1>::Zero();
        v11.segment<7>(0) = joint_velocities;
        v11(9) = pole_positions(1);
        v11(10) = pole_positions(0);

        Eigen::Matrix<double, 11, 11> result_matrix = pinocchio::computeCoriolisMatrix(_pinocchio_model, _pinocchio_data, q11, v11);
        return (result_matrix * v11).segment<7>(0);
    }
    else
    {
        Eigen::Matrix<double, 10, 1> q10 = Eigen::Matrix<double, 10, 1>::Zero();
        q10.segment<7>(0) = joint_positions;
        q10(9) = pole_positions(0);
        Eigen::Matrix<double, 10, 1> v10 = Eigen::Matrix<double, 10, 1>::Zero();
        v10.segment<7>(0) = joint_velocities;
        v10(9) = pole_velocities(0);

        Eigen::Matrix<double, 10, 10> result_matrix = pinocchio::computeCoriolisMatrix(_pinocchio_model, _pinocchio_data, q10, v10);
        return (result_matrix * v10).segment<7>(0);
    }
    
}

Eigen::Matrix<double, 6, 7> franka_pole::FrankaModel::get_effector_jacobian(const Eigen::Matrix<double, 7, 1> &joint_positions)
{
    std::array<double, 7> q7;
    Eigen::Matrix<double, 7, 1>::Map(&q7[0]) = joint_positions;
    std::array<double, 16> gripper_frame({0.7071, -0.7071, 0, 0, 0.7071, 0.7071, 0, 0, 0, 0, 1, 0, 0, 0, 0.1034, 1});
    std::array<double, 16> stiffness_frame({ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 });
    return Eigen::Matrix<double, 6, 7>::Map(_model_handle->getZeroJacobian(franka::Frame::kEndEffector, q7, gripper_frame, stiffness_frame).data());
}

Eigen::Matrix<double, 7, 1> franka_pole::FrankaModel::get_torques(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 7, 1> &joint_velocities, const Eigen::Matrix<double, 2, 1> &pole_positions, const Eigen::Matrix<double, 2, 1> &pole_velocities, const Eigen::Matrix<double, 3, 1> &acceleration)
{
    Eigen::Matrix<double, 6, 7> jacobian = get_effector_jacobian(joint_positions);

    if (_controller->param->two_dimensional())
    {
        Eigen::Matrix<double, 11, 1> q11 = Eigen::Matrix<double, 11, 1>::Zero();
        q11.segment<7>(0) = joint_positions;
        q11(9) = pole_positions(1);
        q11(10) = pole_positions(0);
        Eigen::Matrix<double, 11, 1> v11 = Eigen::Matrix<double, 11, 1>::Zero();
        v11.segment<7>(0) = joint_velocities;
        v11(9) = pole_positions(1);
        v11(10) = pole_positions(0);
        Eigen::Matrix<double, 6, 1> a6 = Eigen::Matrix<double, 6, 1>::Zero();
        a6.segment<3>(0) = acceleration;
        Eigen::Matrix<double, 11, 1> a11 = Eigen::Matrix<double, 11, 1>::Zero();
        a11.segment<7>(0) = pseudo_inverse(jacobian, true) * a6;
        //a11(9) = ???
        //a11(10) = ???
        return pinocchio::rnea(_pinocchio_model, _pinocchio_data, q11, v11, a11).segment<7>(0);
    }
    else
    {
        Eigen::Matrix<double, 10, 1> q10 = Eigen::Matrix<double, 10, 1>::Zero();
        q10.segment<7>(0) = joint_positions;
        q10(9) = pole_positions(0);
        Eigen::Matrix<double, 10, 1> v10 = Eigen::Matrix<double, 10, 1>::Zero();
        v10.segment<7>(0) = joint_velocities;
        v10(9) = pole_velocities(0);
        Eigen::Matrix<double, 6, 1> a6 = Eigen::Matrix<double, 6, 1>::Zero();
        a6.segment<3>(0) = acceleration;
        Eigen::Matrix<double, 10, 1> a10 = Eigen::Matrix<double, 10, 1>::Zero();
        a10.segment<7>(0) = pseudo_inverse(jacobian, true) * a6;
        //a10(9) = ???
        return pinocchio::rnea(_pinocchio_model, _pinocchio_data, q10, v10, a10).segment<7>(0);
    }
}