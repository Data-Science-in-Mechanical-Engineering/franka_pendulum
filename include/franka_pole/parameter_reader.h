#pragma once

#include <franka_pole/model.h>

#include <ros/node_handle.h>
#include <Eigen/Dense>

namespace franka_pole
{
    class ParameterReader
    {
    private:
        ros::NodeHandle &_node_handle;

        std::string _read_string(const std::string name) const;
        bool _read_bool(const std::string name) const;
        unsigned int _read_uint(const std::string name) const;
        double _read_double(const std::string name) const;
        static double _check_double(const std::string name, double value);
        template<int N> Eigen::Matrix<double, N, 1> _read_vector(const std::string name) const;
        template<int N> static bool _nan_vector(const Eigen::Matrix<double, N, 1> &vector);
        template<int N> static Eigen::Matrix<double, N, 1> _check_vector(const std::string name, const Eigen::Matrix<double, N, 1> &vector);
        template<int N> static Eigen::Matrix<double, N, 1> _replace_vector(const Eigen::Matrix<double, N, 1> &dest, const Eigen::Matrix<double, N, 1> &source);
        Eigen::Quaterniond _read_quaternion(const std::string name) const;
        static bool _nan_quaternion(const Eigen::Quaterniond &quaternion);
        static Eigen::Quaterniond _replace_quaternion(const Eigen::Quaterniond &dest, const Eigen::Quaterniond &source);
        static Eigen::Quaterniond _check_quaternion(const std::string name, const Eigen::Quaterniond &quaternion);

    public:
        ParameterReader(ros::NodeHandle &node_handle);

        // Essential
        std::string arm_id() const;
        bool simulated() const;
        Model model() const;

        // Periods
        unsigned int franka_period() const;
        unsigned int pole_period() const;
        unsigned int command_period() const;
        unsigned int publish_period() const;
        unsigned int controller_period() const;

        // Target state and constraints
        Eigen::Matrix<double, 3, 1> target_effector_position() const;
        Eigen::Quaterniond target_effector_orientation() const;
        double target_joint0_position() const;
        Eigen::Matrix<double, 3, 1> min_effector_position() const;
        Eigen::Matrix<double, 3, 1> max_effector_position() const;

        // Initial state
        Eigen::Matrix<double, 3, 1> initial_effector_position() const;
        Eigen::Quaterniond initial_effector_orientation() const;
        double initial_joint0_position() const;
        Eigen::Matrix<double, 2, 1> initial_pole_positions() const;
        Eigen::Matrix<double, 2, 1> initial_pole_velocities() const;

        // Stiffness
        Eigen::Matrix<double, 3, 1> outbound_translation_stiffness() const;
        Eigen::Matrix<double, 3, 1> outbound_translation_damping() const;
        Eigen::Matrix<double, 3, 1> outbound_rotation_stiffness() const;
        Eigen::Matrix<double, 3, 1> outbound_rotation_damping() const;
        Eigen::Matrix<double, 3, 1> translation_stiffness() const;
        Eigen::Matrix<double, 3, 1> translation_damping() const;
        Eigen::Matrix<double, 3, 1> rotation_stiffness() const;
        Eigen::Matrix<double, 3, 1> rotation_damping() const;

        Eigen::Matrix<double, 7, 1> joint_stiffness() const;
        Eigen::Matrix<double, 7, 1> joint_damping() const;

        Eigen::Matrix<double, 7, 1> nullspace_stiffness() const;
        Eigen::Matrix<double, 7, 1> nullspace_damping() const;

        double dynamics() const;

        // Filters
        double pole_angle_filter() const;
        double pole_dangle_filter() const;

        // Noise
        Eigen::Matrix<double, 7, 1> joint_position_standard_deviation() const;
        Eigen::Matrix<double, 7, 1> joint_velocity_standard_deviation() const;
        Eigen::Matrix<double, 2, 1> pole_angle_standard_deviation() const;
        
        // Reset
        double hardware_reset_duration() const;
        Eigen::Matrix<double, 7, 1> hardware_reset_stiffness() const;
        Eigen::Matrix<double, 7, 1> hardware_reset_damping() const;

        // Control
        Eigen::Matrix<double, 8, 1> control() const;
    };
}