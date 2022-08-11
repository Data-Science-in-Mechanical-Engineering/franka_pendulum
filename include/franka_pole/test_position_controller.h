#pragma once

#include <franka_pole/position_controller.h>

namespace franka_pole
{
    ///High-level position controller that returns sinewave as target position and zero as target velocity
    class TestPositionController : public PositionController
    {
    private:
        //Overrides from franka_pole::AccelerationController
        bool _init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        Eigen::Matrix<double, 3, 1> _get_position_level2(const ros::Time &time, const ros::Duration &period) override;
        Eigen::Matrix<double, 3, 1> _get_velocity_level2(const ros::Time &time, const ros::Duration &period) override;

        FRANKA_POLE_CONTROLLER_DECLARATION();
    };
}