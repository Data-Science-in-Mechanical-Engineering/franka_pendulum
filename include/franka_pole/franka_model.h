#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <franka_hw/franka_model_interface.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

namespace franka_pole
{
    class Controller;

    class FrankaModel
    {
    private:
        //Technical
        Controller *_controller;
        bool _ok = false;

        //ROS technical
        std::unique_ptr<franka_hw::FrankaModelHandle> _model_handle;

        // Pinocchio technical
        size_t _pinocchio_joint_ids[7];
        size_t _pinocchio_finger_joint_ids[2];
        size_t _pinocchio_pole_joint_ids[2];
        pinocchio::Model _pinocchio_model;
        pinocchio::Data _pinocchio_data;

    public:
        FrankaModel(Controller *controller, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle);
        bool ok() const;

        Eigen::Matrix<double, 7, 1> get_gravity(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 2, 1> &pole_positions);
        Eigen::Matrix<double, 7, 1> get_coriolis(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 7, 1> &joint_velocities, const Eigen::Matrix<double, 2, 1> &pole_positions, const Eigen::Matrix<double, 2, 1> &pole_velocities);
        Eigen::Matrix<double, 6, 7> get_effector_jacobian(const Eigen::Matrix<double, 7, 1> &joint_positions);
        Eigen::Matrix<double, 7, 1> get_torques(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 7, 1> &joint_velocities, const Eigen::Matrix<double, 2, 1> &pole_positions, const Eigen::Matrix<double, 2, 1> &pole_velocities, const Eigen::Matrix<double, 3, 1> &acceleration);
    };
}