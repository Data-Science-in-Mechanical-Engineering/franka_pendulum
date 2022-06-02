#pragma once

#include <franka_pole/franka_state.h>
#include <franka_pole/pole_state.h>
#include <franka_pole/publisher.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <memory>
#include <string>

namespace franka_pole
{
    //Basic controller, responsible for getting technical ROS staff, initializing and updating components, reading parameters
    class Controller : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface, hardware_interface::EffortJointInterface, hardware_interface::PositionJointInterface, franka_hw::FrankaStateInterface>
    {
    private:
        //Parameters
        std::string _arm_id = "panda";
        bool _simulated = true;
        bool _two_dimensional = true;

    protected:
        //Essential functions for child classes
        bool _controller_init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle);
        void _controller_starting(const ros::Time&);
        void _controller_pre_update(const ros::Time&, const ros::Duration& period);
        void _controller_post_update(const ros::Time&, const ros::Duration& period);

    public:
        //Components
        std::unique_ptr<FrankaState> franka_state;
        std::unique_ptr<PoleState> pole_state;
        std::unique_ptr<Publisher> publisher;

        //Functions
        std::string get_arm_id() const;
        bool is_simulated() const;
        bool is_two_dimensional() const;
    };
}