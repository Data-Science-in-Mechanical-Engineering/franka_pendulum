#pragma once

#include <franka_pole/acceleration_controller.h>
#include <franka_pole/CommandParameters.h>

#include <ros/node_handle.h>
#include <ros/time.h>

namespace franka_pole
{
    class SimpleAccelerationController : public AccelerationController
    {
    private:
        // Commanded control
        double _a = 16.363880157470703 * 10;
        double _b = 9.875003814697266 * 10;
        double _c = 7.015979766845703 * 10;
        double _d = 11.86760425567627 * 10;
        ros::Subscriber _command_subscriber;
        void _command_callback(const franka_pole::CommandParameters::ConstPtr &msg);

    public:
        //Overridden from MultiInterfaceController
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;
    };
}