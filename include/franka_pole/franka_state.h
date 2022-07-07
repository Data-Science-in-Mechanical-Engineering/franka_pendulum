#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <random>

namespace franka_pole
{
    class Controller;

    class FrankaState
    {
    private:
        //Technical
        Controller *_controller;
        bool _simulated = false;
        std::vector<hardware_interface::JointHandle> _joint_handles;
        std::vector<std::normal_distribution<double>> _random_position_distributions;
        std::vector<std::normal_distribution<double>> _random_velocity_distributions;
        std::default_random_engine _random_engine;

        //Timestamp
        double _timestamp = 0.0;

        //Joints
        Eigen::Matrix<double, 7, 1> _raw_joint_positions;
        Eigen::Matrix<double, 7, 1> _joint_positions;
        Eigen::Matrix<double, 7, 1> _joint_velocities;

        //Basic precomputed data
        Eigen::Matrix<double, 3, 1> _effector_position;
        Eigen::Quaterniond _effector_orientation;
        Eigen::Matrix<double, 6, 1> _effector_velocity;

    public:
        FrankaState(Controller *controller, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle);
        void update(const ros::Time &time);
        
        //Time
        double get_timestamp() const;
        
        //Joint space
        Eigen::Matrix<double, 7, 1> get_raw_joint_positions() const;
        Eigen::Matrix<double, 7, 1> get_joint_positions() const;
        Eigen::Matrix<double, 7, 1> get_joint_velocities() const;

        //Cartesian space
        Eigen::Matrix<double, 3, 1> get_effector_position() const;
        Eigen::Quaterniond get_effector_orientation() const;
        Eigen::Matrix<double, 6, 1> get_effector_velocity() const;
        
        void set_torque(const Eigen::Matrix<double, 7, 1> &torque);
    };
}