#pragma once

#include <franka_pole/controller.h>
#include <franka_pole/ControllerParameters.h>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <memory>

namespace franka_pole
{
    class IntegratedAccelerationController : public Controller
    {
    private:
        //Control
        double _a = 16.363880157470703 / 30;
        double _b = 9.875003814697266 / 30;
        double _c = 7.015979766845703 / 30;
        double _d = 11.86760425567627 / 30;
        double _nullspace_stiffness = 0.0;
        double _nullspace_damping = 0.0;
        Eigen::Matrix<double, 6, 6> _cartesian_stiffness = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 6> _cartesian_damping = Eigen::Matrix<double, 6, 6>::Zero();

        void _update(const franka_pole::ControllerParameters::ConstPtr &msg);

    public:
        //Overridden from MultiInterfaceController
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;
    };
}  // namespace franka_pole