#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <ros/node_handle.h>
#include <Eigen/Dense>

namespace franka_pole
{
    class FrankaModel
    {
    private:
        std::string _arm_id;
        bool _two_dimensional = false;
        Eigen::Matrix<double, 7, 1> _joint_position_standard_deviation = Eigen::Matrix<double, 7, 1>::Zero();
        Eigen::Matrix<double, 7, 1> _joint_velocity_standard_deviation = Eigen::Matrix<double, 7, 1>::Zero();

        // Pinocchio technical
        size_t _effector_frame_id = 0;
        size_t _pole_frame_id = 0;
        pinocchio::Model _model;
        pinocchio::Data _data;

    public:
        FrankaModel(ros::NodeHandle &node_handle);

        Eigen::Matrix<double, 7, 1> get_gravity(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_positions);
        Eigen::Matrix<double, 7, 1> get_coriolis(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 7, 1> &joint_velocities, const Eigen::Matrix<double, 2, 1> &pole_joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_velocities);
        Eigen::Matrix<double, 6, 7> get_effector_jacobian(const Eigen::Matrix<double, 7, 1> &joint_positions);
        Eigen::Matrix<double, 7, 1> get_torques(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 7, 1> &joint_velocities, const Eigen::Matrix<double, 2, 1> &pole_joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_velocities, const Eigen::Matrix<double, 3, 1> &acceleration);
        Eigen::Matrix<double, 3, 1> effector_forward_kinematics(const Eigen::Matrix<double, 7, 1> &joint_positions, Eigen::Quaterniond *effector_orientation);
        Eigen::Matrix<double, 3, 1> pole_forward_kinematics(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_positions, Eigen::Quaterniond *pole_orientation);
        Eigen::Matrix<double, 7, 1> inverse_kinematics(const Eigen::Matrix<double, 3, 1> &effector_position, const Eigen::Quaterniond &effector_orientation, double joint0);
    };
}