#pragma once

#include <franka_pole/model.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <random>
#include <mutex>

namespace franka_pole
{
    class Parameters;
    class FrankaModel;
    class FrankaState;
    class Publisher;

    //Provides pole states
    class PoleState
    {
    private:
        //Reserences
        const Parameters *_parameters;
        FrankaModel *_franka_model;
        const FrankaState *_franka_state;
        Publisher *_publisher;
        std::mutex *_mutex;

        //Technical, simulated
        hardware_interface::JointHandle _joint_handles[2];
        std::normal_distribution<double> _random_angle_distributions[2];
        std::default_random_engine _random_engine;

        //Callback for ROS, not simulated
        ros::Subscriber _subscriber;
        void _callback(const geometry_msgs::TransformStamped::ConstPtr &msg);
        
        //Timestamp
        double _timestamp = 0.0;

        //Angles
        bool _first = true;
        Eigen::Matrix<double, 2, 1> _angle = Eigen::Matrix<double, 2, 1>::Zero();
        Eigen::Matrix<double, 2, 1> _dangle = Eigen::Matrix<double, 2, 1>::Zero();
        Eigen::Matrix<double, 2, 1> _joint_angle = Eigen::Matrix<double, 2, 1>::Zero();
        Eigen::Matrix<double, 2, 1> _joint_dangle = Eigen::Matrix<double, 2, 1>::Zero();

    public:
        PoleState(const Parameters *parameters, FrankaModel *franka_model, const FrankaState *franka_state, Publisher *publisher, std::mutex *mutex, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle);
        void update(const ros::Time &time);

        double get_timestamp();
        Eigen::Matrix<double, 2, 1> get_angle();
        Eigen::Matrix<double, 2, 1> get_dangle();
        Eigen::Matrix<double, 2, 1> get_joint_angle();
        Eigen::Matrix<double, 2, 1> get_joint_dangle();
    };
}