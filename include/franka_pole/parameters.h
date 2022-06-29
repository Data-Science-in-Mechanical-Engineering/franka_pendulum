#pragma once

#include <hardware_interface/robot_hw.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

namespace franka_pole
{
    class Controller;

    class Parameters
    {
    private:
        //Technical
        Controller *_controller;
        bool _ok = false;
        
        //Parameters
        std::string _arm_id = "panda";
        bool _simulated = true;
        bool _two_dimensional = false;
        Eigen::Matrix<double, 3, 1> _translation_stiffness;
        Eigen::Matrix<double, 3, 1> _rotation_stiffness;
        Eigen::Matrix<double, 7, 1> _nullspace_stiffness;
        Eigen::Matrix<double, 7, 1> _joint_stiffness;
        Eigen::Matrix<double, 3, 1> _box_center;
        Eigen::Matrix<double, 3, 1> _box_min;
        Eigen::Matrix<double, 3, 1> _box_max;
        Eigen::Matrix<double, 7, 1> _initial_joint_positions;
        Eigen::Matrix<double, 2, 1> _initial_pole_positions;

    public:
        Parameters(Controller *controller, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle);
        bool ok() const;

        std::string arm_id() const;
        bool simulated() const;
        bool two_dimensional() const;

        Eigen::Matrix<double, 3, 1> translation_stiffness() const;
        Eigen::Matrix<double, 3, 1> rotation_stiffness() const;
        Eigen::Matrix<double, 7, 1> nullspace_stiffness() const;
        Eigen::Matrix<double, 7, 1> joint_stiffness() const;
        Eigen::Matrix<double, 3, 1> box_center() const;
        Eigen::Matrix<double, 3, 1> box_min() const;
        Eigen::Matrix<double, 3, 1> box_max() const;
        Eigen::Matrix<double, 7, 1> initial_joint_positions() const;
        Eigen::Matrix<double, 2, 1> initial_pole_positions() const;
    };
}