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

        //Pole
        ///Notifies publisher about pole update
        ///@param timestamp Latest pole update time
        ///@param angle Angles betweeeen the pole and the planes
        ///@param dangle Derivative of the angle betweeeen the pole and the planes
        ///@param joint_angle Angles of revolute joints in the pole
        ///@param joint_dangle Angular velocities of revolute joints in the pole
        void set_pole(
            const ros::Time &timestamp,
            const Eigen::Matrix<double, 2, 1> &angle,
            const Eigen::Matrix<double, 2, 1> &dangle,
            const Eigen::Matrix<double, 2, 1> &joint_angle,
            const Eigen::Matrix<double, 2, 1> &joint_dangle);
        
        //Franka
        ///Notifies publisher about franka position update
        ///@param timestamp Latest franka position update time
        ///@param position Latest franka's end effector position
        ///@param orientation Latest franka's end effector orientation
        ///@param velocity Latest franka's end effector velocity
        ///@param positions Latest franka's joint positions
        ///@param velocities Latest franka's joint velocities
        void set_franka(const ros::Time &timestamp,
            const Eigen::Matrix<double, 3, 1> &position,
            const Eigen::Quaterniond &orientation,
            const Eigen::Matrix<double, 6, 1> &velocity,
            const Eigen::Matrix<double, 7, 1> &positions,
            const Eigen::Matrix<double, 7, 1> &velocities);

        //Controller
        ///Notifies publisher about franka's torque update
        ///@param timestamp Latest franka's torque update time
        ///@param position Latest franka's commanded position
        ///@param orientation Latest franka's commanded orientation
        ///@param velocity Latest franka's commanded velocity
        ///@param acceleration Latest franka's commanded acceleration
        ///@param positions Latest franka's commanded joint positions
        ///@param velocities Latest franka's commanded joint velocities
        ///@param accelerations Latest franka's commanded joint accelerations
        ///@param torque Latest franka's commanded torque
        void set_command(const ros::Time &timestamp,
            const Eigen::Matrix<double, 3, 1> &position,
            const Eigen::Quaterniond &orientation,
            const Eigen::Matrix<double, 6, 1> &velocity,
            const Eigen::Matrix<double, 6, 1> &acceleration,
            const Eigen::Matrix<double, 7, 1> &positions,
            const Eigen::Matrix<double, 7, 1> &velocities,
            const Eigen::Matrix<double, 7, 1> &accelerations,
            const Eigen::Matrix<double, 7, 1> &torques);

        //Reset
        ///Notifies publisher whether the robot is being resetted
        ///@param reset `true` if the robot is being resetted, `false` if not
        void set_reset(bool reset);
    };
}