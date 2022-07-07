#pragma once

#include <franka_pole/position_controller.h>
#include <franka_pole/CommandParameters.h>

namespace franka_pole
{
    class SimplePositionController : public PositionController
    {
    private:
        bool _two_dimensional = false;
        Eigen::Matrix<double, 3, 1> _target_position = Eigen::Matrix<double, 3, 1>::Zero();
        Eigen::Matrix<double, 3, 1> _max_effector_position = Eigen::Matrix<double, 3, 1>::Zero();
        Eigen::Matrix<double, 3, 1> _min_effector_position = Eigen::Matrix<double, 3, 1>::Zero();
        std::array<double, 2> _a;
        std::array<double, 2> _b;
        std::array<double, 2> _c;
        std::array<double, 2> _d;
        ros::Subscriber _command_subscriber;

        void _command_callback(const franka_pole::CommandParameters::ConstPtr &msg);

    public:
        // Overridden from MultiInterfaceController
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;
    };
}