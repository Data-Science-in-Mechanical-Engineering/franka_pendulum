#pragma once

#include <franka_pole/controller.h>

#include <Eigen/Dense>

namespace franka_pole
{
    class Parameters;

    ///Position controller is a mid-level helper controller, responsible for getting position and velocity target from higher level controller and returning torque to low-level controller
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
        #ifdef FRANKA_POLE_VELOCITY_INTERFACE
            Eigen::Matrix<double, 6, 1> _get_velocity_level1(const ros::Time &time, const ros::Duration &period) override;
        #else
            Eigen::Matrix<double, 7, 1> _get_torque_level1(const ros::Time &time, const ros::Duration &period) override;
        #endif

    protected:
        //Interface for higher level controllers
        ///Initializes higher level controllers
        ///@param robot_hw `hardware_interface::RobotHW` object
        ///@param node_handle ROS node handle
        ///@return `true` if initialization was successfull
        virtual bool _init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) = 0;
        ///Gets position command from higher level controllers
        ///@param time Current time
        ///@param period Time from previous update
        ///@return postiion target
        virtual Eigen::Matrix<double, 3, 1> _get_position_level2(const ros::Time &time, const ros::Duration &period) = 0;
        ///Gets velocity command from higher level controllers
        ///@param time Current time
        ///@param period Time from previous update
        ///@return velocity target
        virtual Eigen::Matrix<double, 3, 1> _get_velocity_level2(const ros::Time &time, const ros::Duration &period) = 0;
    };
}