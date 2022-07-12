#pragma once

#include <franka_pole/acceleration_controller.h>
#include <franka_pole/model.h>

namespace franka_pole
{
    class TestAccelerationController : public AccelerationController
    {
    private:
        Model _model = Model::D1;
        
    public:
        //Overridden from MultiInterfaceController
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;
    };
}