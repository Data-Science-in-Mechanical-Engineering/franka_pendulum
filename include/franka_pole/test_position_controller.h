#pragma once

#include <franka_pole/position_controller.h>

#include <ros/node_handle.h>
#include <ros/time.h>

namespace franka_pole
{
    class TestPositionController : public PositionController
    {
    public:
        // Overridden from MultiInterfaceController
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;
    };
}