#pragma once

#include <franka_pole/CommandReset.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>

#include <ros/time.h>
#include <Eigen/Dense>
#include <semaphore.h>
#include <mutex>

namespace franka_pole
{
    class Parameters;
    class FrankaModel;
    class FrankaState;
    class PoleState;
    class Publisher;

    //Basic controller, responsible for getting technical ROS staff, initializing and updating components, setting torque and reset mechanismus
    class Controller : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface, hardware_interface::PositionJointInterface>
    {
    private:
        hardware_interface::RobotHW *_robot_hw;
        ros::NodeHandle _node_handle;

        //Software reset
        ros::Subscriber _reset_subscriber;
        bool _software_reset;
        bool _hardware_reset;
        sem_t *_software_reset_semaphore;
        Eigen::Matrix<double, 7, 1> _hardware_reset_old_positions;
        double _hardware_reset_time;

        //Period
        Eigen::Matrix<double, 7, 1> _torque;
        unsigned int _franka_period_counter;
        unsigned int _pole_period_counter;
        unsigned int _command_period_counter;
        unsigned int _publish_period_counter;

        //Additional parameters
        Eigen::Matrix<double, 7, 1> _initial_joint_positions;

        std::mutex _mutex;
        void _callback(const franka_pole::CommandReset::ConstPtr &msg);
        void _reset();

    protected:
        //System handling
        bool _init_level0(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle);
        void _starting_level0(const ros::Time &time);
        void _update_level0(const ros::Time &time, const ros::Duration &period);

        //Interface for higher level controllers
        virtual bool _init_level1(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) = 0;
        virtual Eigen::Matrix<double, 7, 1> _get_torque_level1(const ros::Time &time, const ros::Duration &period) = 0;

    public:
        //Components
        Parameters *parameters = nullptr;
        FrankaModel *franka_model = nullptr;
        FrankaState *franka_state = nullptr;
        PoleState *pole_state = nullptr;
        Publisher *publisher = nullptr;
        virtual ~Controller();
    };
}

#define FRANKA_POLE_CONTROLLER_DECLARATION() \
public: \
bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override; \
void starting(const ros::Time &time) override; \
void update(const ros::Time &time, const ros::Duration &period) override;

#define FRANKA_POLE_CONTROLLER_IMPLEMENTATION(__NAME__) \
PLUGINLIB_EXPORT_CLASS(__NAME__, controller_interface::ControllerBase) \
bool __NAME__::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) { return franka_pole::Controller::_init_level0(robot_hw, node_handle); } \
void __NAME__::starting(const ros::Time &time) { franka_pole::Controller::_starting_level0(time); } \
void __NAME__::update(const ros::Time &time, const ros::Duration &period) { franka_pole::Controller::_update_level0(time, period); }
