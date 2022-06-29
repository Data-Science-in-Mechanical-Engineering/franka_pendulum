#pragma once

#include <franka_pole/parameters.h>
#include <franka_pole/franka_model.h>
#include <franka_pole/franka_state.h>
#include <franka_pole/pole_state.h>
#include <franka_pole/publisher.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_pole/CommandReset.h>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <semaphore.h>
#include <memory>
#include <string>

namespace franka_pole
{
    //Basic controller, responsible for getting technical ROS staff, initializing and updating components, setting torque and reset mechanismus
    class Controller : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface, hardware_interface::EffortJointInterface, hardware_interface::PositionJointInterface, franka_hw::FrankaStateInterface>
    {
    private:
        //Reset       
        ros::Subscriber _reset_subscriber;
        bool _software_reset = false;
        bool _hardware_reset = false;
        sem_t *_software_reset_semaphore = nullptr;
        Eigen::Matrix<double, 7, 1> _hardware_reset_initial;
        size_t _hardware_reset_counter;

        void _command_reset(const franka_pole::CommandReset::ConstPtr &msg);

    protected:
        //Essential functions for child classes
        bool _controller_init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle);
        void _controller_starting(const ros::Time&);
        void _controller_pre_update(const ros::Time&, const ros::Duration& period);
        void _controller_post_update(const ros::Time&, const ros::Duration& period, const Eigen::Matrix<double, 7, 1> &torque);

    public:
        //Components
        std::unique_ptr<Parameters> param;
        std::unique_ptr<FrankaModel> franka_model;
        std::unique_ptr<FrankaState> franka_state;
        std::unique_ptr<PoleState> pole_state;
        std::unique_ptr<Publisher> publisher;

        //Reset
        bool is_reset() const;
    };
}