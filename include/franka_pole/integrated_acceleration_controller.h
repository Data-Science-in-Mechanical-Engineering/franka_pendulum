#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <franka_pole/controller.h>
#include <franka_pole/CommandParameters.h>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <memory>

namespace franka_pole
{
    class IntegratedAccelerationController : public Controller
    {
    private:
        // Pinocchio technical
        size_t _pinocchio_joint_ids[10];
        pinocchio::Model _pinocchio_model;
        pinocchio::Data _pinocchio_data;
        
        // Basic control
        double _nullspace_stiffness = 0.0;
        double _nullspace_damping = 0.0;
        Eigen::Matrix<double, 6, 6> _cartesian_stiffness = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 6> _cartesian_damping = Eigen::Matrix<double, 6, 6>::Zero();

        // Commanded control
        double _a = 16.363880157470703;
        double _b = 9.875003814697266;
        double _c = 7.015979766845703;
        double _d = 11.86760425567627;
        ros::Subscriber _command_subscriber;
        void _command_callback(const franka_pole::CommandParameters::ConstPtr &msg);

    public:
        //Overridden from MultiInterfaceController
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;
    };
}