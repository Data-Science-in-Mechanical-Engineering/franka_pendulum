#pragma once

#include <franka_pole/Sample.h>
#include <franka_pole/model.h>

#include <sensor_msgs/JointState.h>
#include <ros/node_handle.h>
#include <Eigen/Dense>

namespace franka_pole
{
    ///Class that publishes data to ROS topic
    class Publisher
    {
    private:
        ros::Publisher _sample_publisher;
        ros::Publisher _joint_state_publisher;
        Sample _sample;
        sensor_msgs::JointState _joint_state;
        size_t _counter = 0;
        Model _model = Model::D1;

    public:
        Publisher(ros::NodeHandle &node_handle);
        void publish();

        //Franka
        void set_franka_timestamp(const ros::Time &timestamp);
        void set_franka_joint_positions(const Eigen::Matrix<double, 7, 1> &positions);
        void set_franka_joint_velocities(const Eigen::Matrix<double, 7, 1> &velocities);
        void set_franka_joint_torques(const Eigen::Matrix<double, 7, 1> &torques);
        void set_franka_effector_position(const Eigen::Matrix<double, 3, 1> &position);
        void set_franka_effector_velocity(const Eigen::Matrix<double, 3, 1> &velocity);
        void set_franka_effector_orientation(const Eigen::Quaterniond &orientation);
        
        //Pole
        void set_pole_timestamp(const ros::Time &timestamp);
        void set_pole_angle(const Eigen::Matrix<double, 2, 1> &angle);
        void set_pole_dangle(const Eigen::Matrix<double, 2, 1> &dangle);
        void set_pole_joint_angle(const Eigen::Matrix<double, 2, 1> &angle);
        void set_pole_joint_dangle(const Eigen::Matrix<double, 2, 1> &dangle);

        //Controller
        void set_command_timestamp(const ros::Time &timestamp);
        void set_command_effector_position(const Eigen::Matrix<double, 3, 1> &position);
        void set_command_effector_velocity(const Eigen::Matrix<double, 3, 1> &velocity);
        void set_command_effector_acceleration(const Eigen::Matrix<double, 3, 1> &acceleration);

        //Reset
        void set_reset(bool reset);
    };
}