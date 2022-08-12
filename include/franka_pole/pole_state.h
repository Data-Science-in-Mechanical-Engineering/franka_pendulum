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

    ///Class responsible for obtaining, adding noise, filtering and differenting pole state
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
        bool _first;
        Eigen::Matrix<double, 2, 1> _angle;
        Eigen::Matrix<double, 2, 1> _dangle;
        Eigen::Matrix<double, 2, 1> _joint_angle;
        Eigen::Matrix<double, 2, 1> _joint_dangle;

    public:
        ///Creates pole state object
        ///@param parameters Reference to parameters object
        ///@param franka_model Reference to franka model object
        ///@param franka_state Reference to franka state object
        ///@param publisher Reference to publisher object
        ///@param mutex Reference system-wide mutex
        ///@param robot_hw `hardware_interface::RobotHW` object
        ///@param node_handle ROS node handle
        PoleState(const Parameters *parameters, FrankaModel *franka_model, const FrankaState *franka_state, Publisher *publisher, std::mutex *mutex, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle);
        ///Resets pole state filters
        void reset();
        ///Updates pole state
        ///@param time Current time
        void update(const ros::Time &time);

        ///Returns timestamp of the most recent update
        ///@return timestamp of the most recent update
        double get_timestamp();
        ///Returns angle between the pole and YZ ("around X") and XZ ("around Y") planes. The angle is defined positive when the pole is headed to positive Y or X direction respectively
        ///@return angle between the pole and the planes
        Eigen::Matrix<double, 2, 1> get_angle();
        ///Returns angular velocity of the pole
        ///@return derivative of the pole angle
        Eigen::Matrix<double, 2, 1> get_dangle();
        ///Returns angle of revolute joints of the pole. The angle is defined positive when the pole is headed to positive Y or X direction respectively
        ///@return angle of revolute joints
        Eigen::Matrix<double, 2, 1> get_joint_angle();
        ///Returns angular velocity of the pole
        ///@return derivative of the pole joint angle
        Eigen::Matrix<double, 2, 1> get_joint_dangle();
    };
}