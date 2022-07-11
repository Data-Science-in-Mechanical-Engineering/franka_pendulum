#pragma once

#include <ros/node_handle.h>
#include <Eigen/Dense>

namespace franka_pole
{
    class Parameters
    {
    private:
        ros::NodeHandle &_node_handle;

        std::string _read_string(const std::string name) const;
        bool _read_bool(const std::string name) const;
        unsigned int _read_uint(const std::string name) const;
        double _read_double(const std::string name) const;
        Eigen::Matrix<double, Eigen::Dynamic, 1> _read_vector(const std::string name, size_t dimension) const;
        Eigen::Quaterniond _read_quaternion(const std::string name) const;

    public:
        Parameters(ros::NodeHandle &node_handle);

        std::string arm_id() const;
        bool simulated() const;
        bool two_dimensional() const;

        unsigned int franka_period() const;
        unsigned int pole_period() const;

        Eigen::Matrix<double, 3, 1> target_effector_position() const;
        Eigen::Quaterniond target_effector_orientation() const;
        Eigen::Matrix<double, 3, 1> min_effector_position() const;
        Eigen::Matrix<double, 3, 1> max_effector_position() const;
        
        Eigen::Matrix<double, 3, 1> initial_effector_position() const;
        Eigen::Quaterniond initial_effector_orientation() const;
        double initial_joint0_position() const;
        Eigen::Matrix<double, 2, 1> initial_pole_positions() const;
        Eigen::Matrix<double, 2, 1> initial_pole_velocities() const;

        Eigen::Matrix<double, 3, 1> translation_stiffness() const;
        Eigen::Matrix<double, 3, 1> translation_stiffness_safety() const;
        Eigen::Matrix<double, 3, 1> rotation_stiffness() const;
        Eigen::Matrix<double, 7, 1> nullspace_stiffness() const;
        Eigen::Matrix<double, 7, 1> joint_stiffness() const;

        Eigen::Matrix<double, 7, 1> joint_position_standard_deviation() const;
        Eigen::Matrix<double, 7, 1> joint_velocity_standard_deviation() const;
        double pole_period_standard_deviation() const;
        Eigen::Matrix<double, 2, 1> pole_angle_standard_deviation() const;
    };
}