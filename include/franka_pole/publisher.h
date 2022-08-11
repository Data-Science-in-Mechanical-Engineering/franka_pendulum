#pragma once

#include <franka_pole/Sample.h>

#include <sensor_msgs/JointState.h>
#include <ros/node_handle.h>
#include <Eigen/Dense>

namespace franka_pole
{
    class Parameters;

    ///Publisher, responsible for publishing data to ROS topics
    class Publisher
    {
    private:
        const Parameters *_parameters;

        ros::Publisher _sample_publisher;
        ros::Publisher _joint_state_publisher;
        Sample _sample;
        sensor_msgs::JointState _joint_state;

    public:
        ///Creates publisher
        ///@param parameters Reference to parameters object
        ///@param node_handle ROS node handle
        Publisher(const Parameters *parameters, ros::NodeHandle &node_handle);
        ///Publisher data
        void publish();

        //Franka
        ///Notifies publisher about franka position update
        ///@param timestamp Latest franka position update time
        void set_franka_timestamp(const ros::Time &timestamp);
        ///Notifies publisher about franka's joint positions
        ///@param positions Latest franka's joint positions
        void set_franka_joint_positions(const Eigen::Matrix<double, 7, 1> &positions);
        ///Notifies publisher about franka's joint velocities
        ///@param velocities Latest franka's joint velocities
        void set_franka_joint_velocities(const Eigen::Matrix<double, 7, 1> &velocities);
        ///Notifies publisher about franka's end effector position
        ///@param position Latest franka's end effector position
        void set_franka_effector_position(const Eigen::Matrix<double, 3, 1> &position);
        ///Notifies publisher about franka's end effector velocity
        ///@param velocity Latest franka's end effector velocity
        void set_franka_effector_velocity(const Eigen::Matrix<double, 3, 1> &velocity);
        ///Notifies publisher about franka's end effector orientation
        ///@param orientation Latest franka's end effector orientation
        void set_franka_effector_orientation(const Eigen::Quaterniond &orientation);
        
        //Pole
        ///Notifies publisher about pole update
        ///@param timestamp Latest pole update time
        void set_pole_timestamp(const ros::Time &timestamp);
        ///Notifies publisher about pole's position
        ///@param angle Angles betweeeen the pole and the planes
        void set_pole_angle(const Eigen::Matrix<double, 2, 1> &angle);
        ///Notifies publisher about pole's velocity
        ///@param dangle Derivative of the angle betweeeen the pole and the planes
        void set_pole_dangle(const Eigen::Matrix<double, 2, 1> &dangle);
        ///Notifies publisher about pole's joint angle
        ///@param angle Angles of revolute joints in the pole
        void set_pole_joint_angle(const Eigen::Matrix<double, 2, 1> &angle);
        ///Notifies publisher about pole's joint velocity
        ///@param dangle Angular velocities of revolute joints in the pole
        void set_pole_joint_dangle(const Eigen::Matrix<double, 2, 1> &dangle);

        //Controller
        ///Notifies publisher about franka's torque update
        ///@param timestamp Latest franka's torque update time
        void set_command_timestamp(const ros::Time &timestamp);
        ///Notifies publisher about franka's commanded position
        ///@param position Latest franka's commanded position
        void set_command_effector_position(const Eigen::Matrix<double, 3, 1> &position);
        ///Notifies publisher about franka's commanded velocity
        ///@param velocity Latest franka's commanded velocity
        void set_command_effector_velocity(const Eigen::Matrix<double, 3, 1> &velocity);
        ///Notifies publisher about franka's commanded acceleration
        ///@param acceleration Latest franka's commanded acceleration
        void set_command_effector_acceleration(const Eigen::Matrix<double, 3, 1> &acceleration);
        ///Notifies publisher about franka's commanded torques
        ///@param torques Latest franka's commanded torques
        void set_command_joint_torques(const Eigen::Matrix<double, 7, 1> &torques);

        //Reset
        ///Notifies publisher whether the robot is being resetted
        ///@param reset `true` if the robot is being resetted, `false` if not
        void set_reset(bool reset);
    };
}