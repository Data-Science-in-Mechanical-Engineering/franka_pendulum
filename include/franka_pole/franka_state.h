#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <franka_hw/franka_state_interface.h>

#include <Eigen/Dense>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace franka_pole
{
    class Controller;

    class FrankaState
    {
    private:
        //Technical
        Controller *_controller;
        bool _ok = false;
        std::unique_ptr<franka_hw::FrankaStateHandle> _state_handle;
        std::vector<hardware_interface::JointHandle> _joint_handles;

        //Timestamp
        double _timestamp = 0.0;

        //End effector
        Eigen::Vector3d _effector_position;
        Eigen::Matrix<double, 6, 1> _effector_velocity;
        Eigen::Quaterniond _effector_orientation;
        Eigen::Affine3d _effector_transform;

        //Joints
        Eigen::Matrix<double, 7, 1> _joint_positions;
        Eigen::Matrix<double, 7, 1> _joint_velocities;

    public:
        FrankaState(Controller *controller, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle);
        bool ok() const;
        void update(const ros::Time &time);
        double get_timestamp();

        //End effector       
        Eigen::Vector3d get_effector_position();
        Eigen::Matrix<double, 6, 1> get_effector_velocity();
        Eigen::Quaterniond get_effector_orientation();
        Eigen::Affine3d get_effector_transform();
        Eigen::Matrix<double, 6, 7> get_effector_jacobian();

        //Joints
        Eigen::Matrix<double, 7, 1> get_joint_positions();
        Eigen::Matrix<double, 7, 1> get_joint_velocities();
        Eigen::Matrix<double, 7, 1> get_coriolis();
        void set_torque(const Eigen::Matrix<double, 7, 1> &torque);
    };
}