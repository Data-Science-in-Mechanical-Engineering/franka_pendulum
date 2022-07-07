#include <franka_pole/franka_model.h>
#include <franka_pole/pseudo_inverse.h>
#include <franka_pole/parameters.h>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <ros/package.h>

franka_pole::FrankaModel::FrankaModel(ros::NodeHandle &node_handle)
{
    Parameters parameters(node_handle);
    _arm_id = parameters.arm_id();
    _two_dimensional = parameters.two_dimensional();
    _joint_position_standard_deviation = parameters.joint_position_standard_deviation();
    _joint_velocity_standard_deviation = parameters.joint_velocity_standard_deviation();

    //Pinocchio model
    std::string package_path = ros::package::getPath("franka_pole");
    pinocchio::urdf::buildModel(package_path + (_two_dimensional ? "/robots/franka_pole_2D.urdf" : "/robots/franka_pole.urdf"), _model);
    _data = pinocchio::Data(_model);
    for (size_t i = 0; i < 7; i++)
    {
        std::string name = _arm_id + "_joint" + std::to_string(i + 1);
        if (!_model.existJointName(name)) throw std::runtime_error("FrankaModel: Joint " + name + " not found");
    }
    for (size_t i = 0; i < 2; i++)
    {
        std::string name = _arm_id + "_finger_joint" + std::to_string(i + 1);
        if (!_model.existJointName(name)) throw std::runtime_error("FrankaModel: Joint " + name + " not found");
    }
    if (!_model.existJointName(_arm_id + "_pole_joint_x")) throw std::runtime_error("FrankaModel: Joint " + _arm_id + "_pole_joint_x not found");
    if (_two_dimensional)
    {
        if (!_model.existJointName(_arm_id + "_pole_joint_y")) throw std::runtime_error("FrankaModel: Joint " + _arm_id + "_pole_joint_y not found");
    }
    _effector_frame_id = _model.getFrameId(_arm_id + "_pole_link_lower");
    _pole_frame_id = _model.getFrameId(_arm_id + "_pole_link_upper");

    //Initializing generator
    
}

Eigen::Matrix<double, 7, 1> franka_pole::FrankaModel::get_gravity(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_positions)
{
    Eigen::Matrix<double, 7, 1> torque;
    if (_two_dimensional)
    {
        Eigen::Matrix<double, 12, 1> masses;
        for (size_t i = 0; i < 12; i++) { masses(i) = _model.inertias[i].mass(); _model.inertias[i].mass() = 0.0; }
        _model.inertias[7].mass() = 0.5; //Needs further modelling

        Eigen::Matrix<double, 11, 1> q11 = Eigen::Matrix<double, 11, 1>::Zero();
        q11(9) = pole_joint_positions(1);
        q11(10) = pole_joint_positions(0);
        torque = pinocchio::computeGeneralizedGravity(_model, _data, q11).segment<7>(0);

        for (size_t i = 0; i < 12; i++) _model.inertias[i].mass() = masses(i);
    }
    else
    {
        Eigen::Matrix<double, 11, 1> masses;
        for (size_t i = 0; i < 11; i++) { masses(i) = _model.inertias[i].mass(); _model.inertias[i].mass() = 0.0; }
        _model.inertias[7].mass() = 0.5; //Needs further modelling

        Eigen::Matrix<double, 10, 1> q10 = Eigen::Matrix<double, 10, 1>::Zero();
        q10.segment<7>(0) = joint_positions;
        q10(9) = pole_joint_positions(0);
        torque = pinocchio::computeGeneralizedGravity(_model, _data, q10).segment<7>(0);

        for (size_t i = 0; i < 11; i++) _model.inertias[i].mass() = masses(i);
    }
    return torque;
}

Eigen::Matrix<double, 7, 1> franka_pole::FrankaModel::get_coriolis(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 7, 1> &joint_velocities, const Eigen::Matrix<double, 2, 1> &pole_joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_velocities)
{
    if (_two_dimensional)
    {
        Eigen::Matrix<double, 11, 1> q11 = Eigen::Matrix<double, 11, 1>::Zero();
        q11.segment<7>(0) = joint_positions;
        q11(9) = pole_joint_positions(1);
        q11(10) = pole_joint_positions(0);
        Eigen::Matrix<double, 11, 1> v11 = Eigen::Matrix<double, 11, 1>::Zero();
        v11.segment<7>(0) = joint_velocities;
        v11(9) = pole_joint_positions(1);
        v11(10) = pole_joint_positions(0);

        return (pinocchio::computeCoriolisMatrix(_model, _data, q11, v11) * v11).segment<7>(0);
    }
    else
    {
        Eigen::Matrix<double, 10, 1> q10 = Eigen::Matrix<double, 10, 1>::Zero();
        q10.segment<7>(0) = joint_positions;
        q10(9) = pole_joint_positions(0);
        Eigen::Matrix<double, 10, 1> v10 = Eigen::Matrix<double, 10, 1>::Zero();
        v10.segment<7>(0) = joint_velocities;
        v10(9) = pole_joint_velocities(0);

        return (pinocchio::computeCoriolisMatrix(_model, _data, q10, v10) * v10).segment<7>(0);
    }
}

Eigen::Matrix<double, 6, 7> franka_pole::FrankaModel::get_effector_jacobian(const Eigen::Matrix<double, 7, 1> &joint_positions)
{
    if (_two_dimensional)
    {
        Eigen::Matrix<double, 11, 1> q11 = Eigen::Matrix<double, 11, 1>::Zero();
        q11.segment<7>(0) = joint_positions;
        pinocchio::forwardKinematics(_model, _data, q11);
    }
    else
    {
        Eigen::Matrix<double, 10, 1> q10 = Eigen::Matrix<double, 10, 1>::Zero();
        q10.segment<7>(0) = joint_positions;
        pinocchio::forwardKinematics(_model, _data, q10);
    }

    //pinocchio's ReferenceFrame::WORLD is not consistent with franka's zeroJacobian
    //This algorithm provides same result as zeroJacobian
    pinocchio::SE3 placement = pinocchio::updateFramePlacement(_model, _data, _effector_frame_id);
    Eigen::Matrix<double, 6, 7> jacobian;
    for (size_t i = 0; i < 7; i++)
    {
        jacobian.block<3, 1>(0, i) = _data.oMi[i+1].rotation_impl().col(2).cross(placement.translation_impl() - _data.oMi[i+1].translation_impl());
        jacobian.block<3, 1>(3, i) = _data.oMi[i+1].rotation_impl().col(2);
    }
    return jacobian;
}

Eigen::Matrix<double, 7, 1> franka_pole::FrankaModel::get_torques(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 7, 1> &joint_velocities, const Eigen::Matrix<double, 2, 1> &pole_joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_velocities, const Eigen::Matrix<double, 3, 1> &acceleration)
{
    pinocchio::Motion gravity = _model.gravity;
    _model.gravity = pinocchio::Motion::Zero();
    Eigen::Matrix<double, 6, 7> jacobian = get_effector_jacobian(joint_positions);
    Eigen::Matrix<double, 7, 1> torques;

    if (_two_dimensional)
    {
        Eigen::Matrix<double, 11, 1> q11 = Eigen::Matrix<double, 11, 1>::Zero();
        q11.segment<7>(0) = joint_positions;
        q11(9) = pole_joint_positions(1);
        q11(10) = pole_joint_positions(0);
        Eigen::Matrix<double, 11, 1> v11 = Eigen::Matrix<double, 11, 1>::Zero();
        v11.segment<7>(0) = joint_velocities;
        v11(9) = pole_joint_velocities(1);
        v11(10) = pole_joint_velocities(0);
        Eigen::Matrix<double, 6, 1> a6 = Eigen::Matrix<double, 6, 1>::Zero();
        a6.segment<3>(0) = acceleration;
        Eigen::Matrix<double, 11, 1> a11 = Eigen::Matrix<double, 11, 1>::Zero();
        a11.segment<7>(0) = pseudo_inverse(jacobian, true) * a6;
        //a11(9) = ???
        //a11(10) = ???
        torques = pinocchio::rnea(_model, _data, q11, v11, a11).segment<7>(0);
    }
    else
    {
        Eigen::Matrix<double, 10, 1> q10 = Eigen::Matrix<double, 10, 1>::Zero();
        q10.segment<7>(0) = joint_positions;
        q10(9) = pole_joint_positions(0);
        Eigen::Matrix<double, 10, 1> v10 = Eigen::Matrix<double, 10, 1>::Zero();
        v10.segment<7>(0) = joint_velocities;
        v10(9) = pole_joint_velocities(0);
        Eigen::Matrix<double, 6, 1> a6 = Eigen::Matrix<double, 6, 1>::Zero();
        a6.segment<3>(0) = acceleration;
        Eigen::Matrix<double, 10, 1> a10 = Eigen::Matrix<double, 10, 1>::Zero();
        a10.segment<7>(0) = pseudo_inverse(jacobian, true) * a6;
        //a10(9) = ???
        torques = pinocchio::rnea(_model, _data, q10, v10, a10).segment<7>(0);
    }

    _model.gravity = gravity;
    return torques;
}

Eigen::Matrix<double, 3, 1> franka_pole::FrankaModel::effector_forward_kinematics(const Eigen::Matrix<double, 7, 1> &joint_positions, Eigen::Quaterniond *effector_orientation)
{
    if (_two_dimensional)
    {
        Eigen::Matrix<double, 11, 1> q11 = Eigen::Matrix<double, 11, 1>::Zero();
        q11.segment<7>(0) = joint_positions;
        pinocchio::forwardKinematics(_model, _data, q11);
    }
    else
    {
        Eigen::Matrix<double, 10, 1> q10 = Eigen::Matrix<double, 10, 1>::Zero();
        q10.segment<7>(0) = joint_positions;
        pinocchio::forwardKinematics(_model, _data, q10);
    }
    pinocchio::SE3 placement = pinocchio::updateFramePlacement(_model, _data, _effector_frame_id);
    if (effector_orientation != nullptr) *effector_orientation = placement.rotation_impl();
    return placement.translation_impl();
}

Eigen::Matrix<double, 3, 1> franka_pole::FrankaModel::pole_forward_kinematics(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_positions, Eigen::Quaterniond *pole_orientation)
{
    if (_two_dimensional)
    {
        Eigen::Matrix<double, 11, 1> q11 = Eigen::Matrix<double, 11, 1>::Zero();
        q11.segment<7>(0) = joint_positions;
        q11(9) = pole_joint_positions(1);
        q11(10) = pole_joint_positions(0);
        pinocchio::forwardKinematics(_model, _data, q11);
    }
    else
    {
        Eigen::Matrix<double, 10, 1> q10 = Eigen::Matrix<double, 10, 1>::Zero();
        q10.segment<7>(0) = joint_positions;
        q10(9) = pole_joint_positions(0);
        pinocchio::forwardKinematics(_model, _data, q10);
    }
    pinocchio::SE3 placement = pinocchio::updateFramePlacement(_model, _data, _pole_frame_id);
    if (pole_orientation != nullptr) *pole_orientation = placement.rotation_impl();
    return placement.translation_impl();
}

Eigen::Matrix<double, 7, 1> franka_pole::FrankaModel::inverse_kinematics(const Eigen::Matrix<double, 3, 1> &effector_position, const Eigen::Quaterniond &effector_orientation, double joint0)
{
    //Copying hints
    const double hint[] = { 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4 };
    Eigen::VectorXd result(_model.nq);
    result.setZero();
    result(0) = joint0;
    for (size_t i = 1; i < 7; i++) result(i) = hint[i];

    //Constants
    const pinocchio::SE3 goal(effector_orientation, effector_position);
    const double tolerance   = 1e-4;
    const int max_iteration  = 1000;
    const double step        = 1e-1;
    const double damp        = 1e-6;

    //Preallocation
    pinocchio::Data::Matrix6x jacobian(6, _model.nv);
    jacobian.setZero();
    Eigen::VectorXd gradient(_model.nv);
    pinocchio::Data::Matrix6 jacobian2;

    //Loop
    for (size_t i = 0; i < max_iteration; i++)
    {
        pinocchio::forwardKinematics(_model, _data, result);
        pinocchio::SE3 placement = pinocchio::updateFramePlacement(_model, _data, _effector_frame_id);
        pinocchio::SE3 difference = goal.actInv(placement);
        Eigen::Matrix<double, 6, 1> error = pinocchio::log6(difference).toVector();
        if (error.norm() < tolerance) return result.segment<7>(0);
        //Originally it were computeJointJacobian, but computeJointJacobian is inconsistent with absolutely everything else (including computeFrameJacobian)
        pinocchio::computeFrameJacobian(_model, _data, result, _effector_frame_id, pinocchio::ReferenceFrame::LOCAL, jacobian);
        jacobian2.noalias() = jacobian * jacobian.transpose();
        jacobian2.diagonal().array() += damp;
        gradient.noalias() = -jacobian.transpose() * jacobian2.ldlt().solve(error);
        result = pinocchio::integrate(_model, result, gradient * step);
        result(0) = joint0;
    }
    throw std::runtime_error("franka_pole::FrankaModel::inverse_kinematics(): Number of iterations exeeded");
}