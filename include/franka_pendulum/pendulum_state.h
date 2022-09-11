#pragma once

#include <franka_pendulum/model.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <random>
#include <mutex>

namespace franka_pendulum
{
    class Parameters;
    class FrankaModel;
    class FrankaState;
    class Publisher;

    ///Class responsible for obtaining, adding noise, filtering and differenting pendulum state
    class PendulumState
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
        double _timestamp;

        //Angles
        bool _first;
        Eigen::Matrix<double, 2, 1> _angle;
        Eigen::Matrix<double, 2, 1> _dangle;
        Eigen::Matrix<double, 2, 1> _joint_angle;
        Eigen::Matrix<double, 2, 1> _joint_dangle;

        //Generalized processing from update & _callback
        void _update(const Eigen::Quaterniond &pendulum_orientation, ros::Time timestamp);

    public:
        ///Creates pendulum state object
        ///@param parameters Reference to parameters object
        ///@param franka_model Reference to franka model object
        ///@param franka_state Reference to franka state object
        ///@param publisher Reference to publisher object
        ///@param mutex Reference system-wide mutex
        ///@param robot_hw `hardware_interface::RobotHW` object
        ///@param node_handle ROS node handle
        PendulumState(const Parameters *parameters, FrankaModel *franka_model, const FrankaState *franka_state, Publisher *publisher, std::mutex *mutex, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle);
        ///Resets pendulum state filters
        ///@param joint_angle Angles of revolute joints in the pendulum
        ///@param joint_dangle Angular velocities of revolute joints in the pendulum
        void reset(const Eigen::Matrix<double, 2, 1> &joint_angle, const Eigen::Matrix<double, 2, 1> &joint_dangle);
        ///Updates pendulum state
        ///@param time Current time
        void update(const ros::Time &time);

        ///Returns timestamp of the most recent update
        ///@return timestamp of the most recent update
        double get_timestamp();
        ///Returns angle between the pendulum and YZ ("around X") and XZ ("around Y") planes. The angle is defined positive when the pendulum is headed to positive Y or X direction respectively
        ///@return angle between the pendulum and the planes
        Eigen::Matrix<double, 2, 1> get_angle();
        ///Returns angular velocity of the pendulum
        ///@return derivative of the pendulum angle
        Eigen::Matrix<double, 2, 1> get_dangle();
        ///Returns angle of revolute joints of the pendulum. The angle is defined positive when the pendulum is headed to positive Y or X direction respectively
        ///@return angle of revolute joints
        Eigen::Matrix<double, 2, 1> get_joint_angle();
        ///Returns angular velocity of the pendulum
        ///@return derivative of the pendulum joint angle
        Eigen::Matrix<double, 2, 1> get_joint_dangle();
    };
}