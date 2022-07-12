#pragma once

#include <franka_pole/position_controller.h>
#include <franka_pole/model.h>

#include <Eigen/Dense>

namespace franka_pole
{
    class TestPositionController : public PositionController
    {
    private:
        Model _model = Model::D1;
        Eigen::Matrix<double, 3, 1> _target_position = Eigen::Matrix<double, 3, 1>::Zero();

    public:
        // Overridden from MultiInterfaceController
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        void starting(const ros::Time &time) override;
        void update(const ros::Time &time, const ros::Duration &period) override;
    };
}