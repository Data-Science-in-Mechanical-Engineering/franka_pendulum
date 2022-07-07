#pragma once

#include <franka_pole/position_controller.h>
#include <franka_pole/CommandPosition.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

namespace franka_pole
{
    class ExternalPositionController : public PositionController
    {
    private:        
        // Commanded control
        Eigen::Matrix<double, 3, 1> _position_target = Eigen::Matrix<double, 3, 1>::Zero();
        Eigen::Matrix<double, 3, 1> _velocity_target = Eigen::Matrix<double, 3, 1>::Zero();
        ros::Subscriber _command_subscriber;
        void _command_callback(const franka_pole::CommandPosition::ConstPtr &msg);

    public:
        //Overridden from MultiInterfaceController
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;
    };
}