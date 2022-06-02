#pragma once

#include <franka_pole/acceleration_controller.h>
#include <franka_pole/CommandAcceleration.h>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

namespace franka_pole
{
    class ExternalAccelerationController : public AccelerationController
    {
    private:        
        // Commanded control
        Eigen::Matrix<double, 3, 1> _acceleration_target = Eigen::Matrix<double, 3, 1>::Zero();
        ros::Subscriber _command_subscriber;
        void _command_callback(const franka_pole::CommandAcceleration::ConstPtr &msg);

    public:
        //Overridden from MultiInterfaceController
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;
    };
}