#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <Eigen/Dense>

namespace franka_pole
{
    class Parameters;

    class FrankaModel
    {
    private:
        //References
        const Parameters *_parameters;

        //Pinocchio technical
        size_t _effector_frame_id;
        size_t _pole_frame_id;
        mutable pinocchio::Model _model;
        mutable pinocchio::Data _data;

    public:
        FrankaModel(const Parameters *parameters);
        Eigen::Matrix<double, 9, 1> get_gravity9(const Eigen::Matrix<double, 7, 1> &joint_positions) const;
        Eigen::Matrix<double, 10, 1> get_gravity10(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_positions) const;
        Eigen::Matrix<double, 11, 1> get_gravity11(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_positions) const;
        Eigen::Matrix<double, 9, 1> get_coriolis9(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 7, 1> &joint_velocities) const;
        Eigen::Matrix<double, 10, 1> get_coriolis10(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 7, 1> &joint_velocities, const Eigen::Matrix<double, 2, 1> &pole_joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_velocities) const;
        Eigen::Matrix<double, 11, 1> get_coriolis11(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 7, 1> &joint_velocities, const Eigen::Matrix<double, 2, 1> &pole_joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_velocities) const;
        Eigen::Matrix<double, 9, 9> get_mass_matrix9(const Eigen::Matrix<double, 7, 1> &joint_positions) const;
        Eigen::Matrix<double, 10, 10> get_mass_matrix10(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_positions) const;
        Eigen::Matrix<double, 11, 11> get_mass_matrix11(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_positions) const;
        Eigen::Matrix<double, 6, 7> get_effector_jacobian(const Eigen::Matrix<double, 7, 1> &joint_positions) const;
        Eigen::Matrix<double, 3, 1> get_effector_centroidal_acceleration(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 7, 1> &joint_velocities) const;
        Eigen::Matrix<double, 3, 1> effector_forward_kinematics(const Eigen::Matrix<double, 7, 1> &joint_positions, Eigen::Quaterniond *effector_orientation) const;
        Eigen::Matrix<double, 3, 1> pole_forward_kinematics(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_positions, Eigen::Quaterniond *pole_orientation) const;
        Eigen::Matrix<double, 7, 1> effector_inverse_kinematics(const Eigen::Matrix<double, 3, 1> &effector_position, const Eigen::Quaterniond &effector_orientation, double joint0) const;
    };
}