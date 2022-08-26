#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#ifdef FRANKA_POLE_VELOCITY_INTERFACE
    #include <franka_hw/franka_cartesian_command_interface.h>
#endif
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <random>
#include <memory>

namespace franka_pole
{
    class Controller;
    class Parameters;
    class FrankaModel;
    class Publisher;
    class PoleState;

    ///Class responsible for obtaining, adding noise and doing basic calculations to franka state
    class FrankaState
    {
    friend Controller;
    friend PoleState;
    private:
        //References
        const Parameters *_parameters;
        FrankaModel *_franka_model;
        Publisher *_publisher;

        //Technical
        #ifdef FRANKA_POLE_VELOCITY_INTERFACE
            std::unique_ptr<franka_hw::FrankaStateHandle> _state_handle;
            std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> _velocity_handle;
        #else
            hardware_interface::JointHandle _joint_handles[7];
        #endif
        std::normal_distribution<double> _random_position_distributions[7];
        std::normal_distribution<double> _random_velocity_distributions[7];
        std::default_random_engine _random_engine;

        //Timestamp
        double _timestamp;

        //Joints
        Eigen::Matrix<double, 7, 1> _exact_joint_positions;
        Eigen::Matrix<double, 7, 1> _joint_positions;
        Eigen::Matrix<double, 7, 1> _joint_velocities;

        //Basic precomputed data
        Eigen::Matrix<double, 3, 1> _effector_position;
        Eigen::Quaterniond _effector_orientation;
        Eigen::Matrix<double, 6, 1> _effector_velocity;

        #ifdef FRANKA_POLE_VELOCITY_INTERFACE
            void _set_velocity(const Eigen::Matrix<double, 6, 1> &velocity);
        #else
            void _set_torque(const Eigen::Matrix<double, 7, 1> &torque);
        #endif

    public:
        ///Creates franka's state object
        ///@param parameters Reference to parameters object
        ///@param franka_model Reference to franka model object
        ///@param publisher Reference to publisher object
        ///@param robot_hw `hardware_interface::RobotHW` object
        FrankaState(const Parameters *parameters, FrankaModel *franka_model, Publisher *publisher, hardware_interface::RobotHW *robot_hw);
        ///Resets franka state
        void reset();
        ///Updates robot's state
        ///@param time Current time
        void update(const ros::Time &time);
        
        ///Returns timestamp of the most recent update
        ///@return timestamp of the most recent update
        double get_timestamp() const;
        ///Returns joint positions of the robot
        ///@return joint posiitons
        Eigen::Matrix<double, 7, 1> get_joint_positions() const;
        ///Returns joint velocities of the robot
        ///@return joint angular velocities
        Eigen::Matrix<double, 7, 1> get_joint_velocities() const;
        ///Returns end effector position
        ///@return end effector position
        Eigen::Matrix<double, 3, 1> get_effector_position() const;
        ///Returns end effector orientation
        ///@return end effector orientation
        Eigen::Quaterniond get_effector_orientation() const;
        ///Returns end effector velocity (translational and rotational)
        ///@return end effector velocity
        Eigen::Matrix<double, 6, 1> get_effector_velocity() const;
    };
}