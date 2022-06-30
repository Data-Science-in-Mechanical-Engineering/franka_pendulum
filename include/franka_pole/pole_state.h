#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

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
        Eigen::Matrix<double, 2, 1> _joint_angle;
        Eigen::Matrix<double, 2, 1> _joint_dangle;
        Eigen::Matrix<double, 2, 1> _angle;

        //Callback for ROS, used if simulated
        ros::Subscriber _pose_stamped_subscriber;
        void _update(const geometry_msgs::PoseStamped::ConstPtr &msg);

    public:
        PoleState(Controller *controller, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle);
        bool ok() const;
        void update(const ros::Time &time);
        
        double get_timestamp() const;
        Eigen::Matrix<double, 2, 1> get_joint_angle() const;
        Eigen::Matrix<double, 2, 1> get_joint_dangle() const;
        Eigen::Matrix<double, 2, 1> get_angle() const;
    };
}