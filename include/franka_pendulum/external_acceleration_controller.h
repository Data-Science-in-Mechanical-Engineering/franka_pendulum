#pragma once

#include <franka_pendulum/acceleration_controller.h>
#include <franka_pendulum/CommandAcceleration.h>

#include <Eigen/Dense>
#include <mutex>

namespace franka_pendulum
{
    ///High-level acceleration controller that returns acceleration set by ROS topic as target acceleration
    class ExternalAccelerationController : public AccelerationController
    {
    private:        
        //Subscriber
        Eigen::Matrix<double, 3, 1> _acceleration_target;
        ros::Subscriber _subscriber;
        bool _subscribed = false;
        void _callback(const franka_pendulum::CommandAcceleration::ConstPtr &msg);

        //Overrides from franka_pendulum::AccelerationController
        bool _init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        Eigen::Matrix<double, 3, 1> _get_acceleration_level2(const ros::Time &time, const ros::Duration &period) override;

        FRANKA_POLE_CONTROLLER_DECLARATION();
    };
}