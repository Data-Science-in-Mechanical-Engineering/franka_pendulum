#pragma once

#include <franka_pendulum/acceleration_controller.h>

namespace franka_pendulum
{
    ///Acceleration controller that reads state (effector position, effector velocity, pendulum inclination, pendulum angular velocity), multiplies it by gain ("control" parameter) and returns as acceleration
    class SimpleAccelerationController : public AccelerationController
    {
    private:
        //Local time
        ros::Time _time = ros::Time(0,0);

        //Overrides from franka_pendulum::AccelerationController
        bool _init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        Eigen::Matrix<double, 3, 1> _get_acceleration_level2(const ros::Time &time, const ros::Duration &period) override;

        FRANKA_POLE_CONTROLLER_DECLARATION();
    };
}