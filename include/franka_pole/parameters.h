#pragma once

#include <franka_pole/model.h>
#include <franka_pole/CommandParameters.h>

#include <ros/ros.h>
#include <Eigen/Dense>

namespace franka_pole
{
    class ParameterReader;

    class Parameters
    {
    private:
        ros::Subscriber _subscriber;
        void _callback(const CommandParameters::ConstPtr &msg);
        static void _replace_uint(unsigned int *dest, unsigned int source);
        static void _replace_double(double *dest, double source);
        static void _replace_quaternion(Eigen::Quaterniond *dest, const boost::array<double, 4> &source);
        template<int N> static void _replace_vector(Eigen::Matrix<double, N, 1> *dest, const boost::array<double, N> &source);

    public:
        Parameters(const ParameterReader &reader, ros::NodeHandle &node_handle);
        
        // Essential
        std::string arm_id;
        bool simulated;
        Model model;

        // Periods
        unsigned int franka_period;
        unsigned int pole_period;
        unsigned int command_period;
        unsigned int publish_period;
        unsigned int controller_period;

        // Target state and constraints
        Eigen::Matrix<double, 3, 1> target_effector_position;
        Eigen::Quaterniond target_effector_orientation;
        double target_joint0_position;
        Eigen::Matrix<double, 3, 1> min_effector_position;
        Eigen::Matrix<double, 3, 1> max_effector_position;

        // Initial state
        Eigen::Matrix<double, 3, 1> initial_effector_position;
        Eigen::Quaterniond initial_effector_orientation;
        double initial_joint0_position;
        Eigen::Matrix<double, 2, 1> initial_pole_positions;
        Eigen::Matrix<double, 2, 1> initial_pole_velocities;

        // Stiffness
        Eigen::Matrix<double, 3, 1> outbound_translation_stiffness;
        Eigen::Matrix<double, 3, 1> outbound_translation_damping;
        Eigen::Matrix<double, 3, 1> outbound_rotation_stiffness;
        Eigen::Matrix<double, 3, 1> outbound_rotation_damping;
        Eigen::Matrix<double, 3, 1> translation_stiffness;
        Eigen::Matrix<double, 3, 1> translation_damping;
        Eigen::Matrix<double, 3, 1> rotation_stiffness;
        Eigen::Matrix<double, 3, 1> rotation_damping;

        Eigen::Matrix<double, 7, 1> joint_stiffness;
        Eigen::Matrix<double, 7, 1> joint_damping;

        Eigen::Matrix<double, 7, 1> nullspace_stiffness;
        Eigen::Matrix<double, 7, 1> nullspace_damping;

        double dynamics;

        // Filters
        double pole_angle_filter;
        double pole_dangle_filter;

        // Noise
        Eigen::Matrix<double, 7, 1> joint_position_standard_deviation;
        Eigen::Matrix<double, 7, 1> joint_velocity_standard_deviation;
        Eigen::Matrix<double, 2, 1> pole_angle_standard_deviation;

        // Reset
        double hardware_reset_duration;
        Eigen::Matrix<double, 7, 1> hardware_reset_stiffness;
        Eigen::Matrix<double, 7, 1> hardware_reset_damping;

        // Control
        Eigen::Matrix<double, 8, 1> control;
    };
}