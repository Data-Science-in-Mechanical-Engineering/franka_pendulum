#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/node_handle.h>
#include <ros/time.h>

namespace franka_pole
{
    class Controller;

    //Provides pole states
    class PoleState
    {
    private:
        //Technical
        Controller *_controller = nullptr;
        bool _ok = false;
        std::vector<hardware_interface::JointHandle> _joint_handles;
        
        //Timestamp
        double _timestamp = 0.0;

        //Angles
        double _angle = 0.0;
        double _dangle = 0.0;

        //Callback for ROS, used if simulated
        ros::Subscriber _pose_stamped_subscriber;
        void _update(const geometry_msgs::PoseStamped::ConstPtr &msg);

    public:
        PoleState(Controller *controller, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle);
        bool ok();
        void update(const ros::Time &time);
        double get_timestamp();

        double get_angle();
        double get_dangle();
    };
}