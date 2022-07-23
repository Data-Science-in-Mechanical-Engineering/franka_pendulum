#pragma once

#include <franka_pole/model.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <random>
#include <type_traits>

namespace franka_pole
{
    class Controller;

    //Provides pole states
    class PoleState
    {
    private:
        //Technical
        Controller *_controller = nullptr;
        Model _model = Model::D1;
        bool _simulated = false;
        std::vector<hardware_interface::JointHandle> _joint_handles;
        std::vector<std::normal_distribution<double>> _random_angle_distributions;
        std::default_random_engine _random_engine;
        
        //Timestamp
        double _timestamp = 0.0;

        //Angles
        Eigen::Matrix<double, 2, 1> _angle = std::numeric_limits<double>::quiet_NaN() * Eigen::Matrix<double, 2, 1>::Zero();
        Eigen::Matrix<double, 2, 1> _dangle = Eigen::Matrix<double, 2, 1>::Zero();
        Eigen::Matrix<double, 2, 1> _joint_angle = std::numeric_limits<double>::quiet_NaN() * Eigen::Matrix<double, 2, 1>::Zero();
        Eigen::Matrix<double, 2, 1> _joint_dangle = Eigen::Matrix<double, 2, 1>::Zero();

        //Callback for ROS, used if simulated
        ros::Subscriber _pose_stamped_subscriber;
        void _update(const geometry_msgs::TransformStamped::ConstPtr &msg);

    public:
        PoleState(Controller *controller, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle);
        void update(const ros::Time &time);
        
        double get_timestamp() const;
        Eigen::Matrix<double, 2, 1> get_angle() const;
        Eigen::Matrix<double, 2, 1> get_dangle() const;
        Eigen::Matrix<double, 2, 1> get_joint_angle() const;
        Eigen::Matrix<double, 2, 1> get_joint_dangle() const;
    };
}