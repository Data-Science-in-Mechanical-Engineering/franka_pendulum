#pragma once

#include <franka_pole/controller.h>

#include <Eigen/Dense>

namespace franka_pole
{
    class PositionController : public Controller
    {
    private:
        // Basic control
        Eigen::Matrix<double, 6, 6> _cartesian_stiffness = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 6> _cartesian_damping = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 7, 1> _nullspace_stiffness = Eigen::Matrix<double, 7, 1>::Zero();
        Eigen::Quaterniond _orientation_target = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);
        Eigen::Matrix<double, 7, 1> _target_joint_positions = Eigen::Matrix<double, 7, 1>::Zero();

    protected:
        //Essential functions for child classes
        bool _controller_init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle);
        void _controller_starting(const ros::Time &time);
        void _controller_pre_update(const ros::Time &time, const ros::Duration &period);
        void _controller_post_update(const ros::Time &time, const ros::Duration &period, const Eigen::Matrix<double, 3, 1> &position_target, const Eigen::Matrix<double, 3, 1> &velocity_target);
    };
}