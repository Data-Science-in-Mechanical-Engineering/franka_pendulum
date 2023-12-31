#pragma once

#include <franka_pendulum/position_controller.h>
#include <franka_pendulum/CommandPosition.h>

#include <Eigen/Dense>
#include <mutex>

namespace franka_pendulum
{
    ///High-level position controller that returns position and acceleration set by ROS topic as target position and acceleration
    class ExternalPositionController : public PositionController
    {
    private:
        //Subscriber
        Eigen::Matrix<double, 3, 1> _position_target = Eigen::Matrix<double, 3, 1>::Zero();
        Eigen::Matrix<double, 3, 1> _velocity_target = Eigen::Matrix<double, 3, 1>::Zero();
        ros::Subscriber _subscriber;
        bool _subscribed = false;
        void _callback(const franka_pendulum::CommandPosition::ConstPtr &msg);

        //Overrides from franka_pendulum::PositionController
        bool _init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        Eigen::Matrix<double, 3, 1> _get_position_level2(const ros::Time &time, const ros::Duration &period) override;
        Eigen::Matrix<double, 3, 1> _get_velocity_level2(const ros::Time &time, const ros::Duration &period) override;

        FRANKA_POLE_CONTROLLER_DECLARATION();
    };
}