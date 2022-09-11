#pragma once

#include <franka_pendulum/position_controller.h>

namespace franka_pendulum
{
    ///High-level position controller that returns sinewave as target position and zero as target velocity
    class TestPositionController : public PositionController
    {
    private:
        //Local time
        ros::Time _time = ros::Time(0,0);
        
        //Overrides from franka_pendulum::AccelerationController
        bool _init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        Eigen::Matrix<double, 3, 1> _get_position_level2(const ros::Time &time, const ros::Duration &period) override;
        Eigen::Matrix<double, 3, 1> _get_velocity_level2(const ros::Time &time, const ros::Duration &period) override;

        FRANKA_POLE_CONTROLLER_DECLARATION();
    };
}