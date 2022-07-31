#pragma once

#include <franka_pole/controller.h>

#include <Eigen/Dense>

namespace franka_pole
{
    class Parameters;

    class PositionController : public Controller
    {
    private:
        //Period
        Eigen::Matrix<double, 3, 1> _position_target;
        Eigen::Matrix<double, 3, 1> _velocity_target;
        unsigned int _controller_period_counter;

        //Additional parameters
        Eigen::Matrix<double, 7, 1> _target_joint_positions;

        //Overrides from franka_pole::Controller
        bool _init_level1(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        Eigen::Matrix<double, 7, 1> _get_torque_level1(const ros::Time &time, const ros::Duration &period) override;

    protected:
        //Interface for higher level controllers
        virtual bool _init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) = 0;
        virtual Eigen::Matrix<double, 3, 1> _get_position_level2(const ros::Time &time, const ros::Duration &period) = 0;
        virtual Eigen::Matrix<double, 3, 1> _get_velocity_level2(const ros::Time &time, const ros::Duration &period) = 0;
    };
}