#pragma once

#include <franka_pole/controller.h>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

namespace franka_pole
{
    class PositionController : public Controller
    {
    private:
        // Basic control
        Eigen::Matrix<double, 6, 6> _cartesian_stiffness = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 6> _cartesian_damping = Eigen::Matrix<double, 6, 6>::Zero();

    protected:
        //Essential functions for child classes
        bool _controller_init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle);
        void _controller_starting(const ros::Time &time);
        void _controller_pre_update(const ros::Time &time, const ros::Duration &period);
        void _controller_post_update(const ros::Time &time, const ros::Duration &period, const Eigen::Matrix<double, 3, 1> &position_target, const Eigen::Matrix<double, 3, 1> &velocity_target);
    };
}