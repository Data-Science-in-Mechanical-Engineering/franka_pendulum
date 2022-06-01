#pragma once

#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace franka_pole
{
    class Controller;

    ///Class that publishes data to ROS topic
    class Publisher
    {
    private:
        //Technical
        ros::Publisher _publisher;
        bool _ok = false;
        
        //Franka
        double _franka_timestamp = 0.0;
        Eigen::Matrix<double, 3, 1> _franka_effector_position = Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0);
        Eigen::Matrix<double, 3, 1> _franka_effector_velocity = Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0);
        Eigen::Quaterniond _franka_effector_orientation = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);
        
        //Pole
        double _pole_timestamp = 0.0;
        double _pole_angle = 0.0;
        double _pole_dangle = 0.0;

        //Controller
        double _control_timestamp = 0.0;
        Eigen::Matrix<double, 3, 1> _control_effector_position = Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0);
        Eigen::Matrix<double, 3, 1> _control_effector_velocity = Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0);
        Eigen::Matrix<double, 3, 1> _control_effector_acceleration = Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0);

    public:
        Publisher(Controller *controller, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle);
        bool ok() const;
        void publish();

        //Franka
        void set_franka_timestamp(const ros::Time &timestamp);
        void set_franka_effector_position(const Eigen::Matrix<double, 3, 1> &position);
        void set_franka_effector_velocity(const Eigen::Matrix<double, 3, 1> &velocity);
        void set_franka_effector_orientation(const Eigen::Quaterniond &orientation);
        
        //Pole
        void set_pole_timestamp(const ros::Time &timestamp);
        void set_pole_angle(double angle);
        void set_pole_dangle(double dangle);

        //Controller
        void set_control_timestamp(const ros::Time &timestamp);
        void set_control_effector_position(const Eigen::Matrix<double, 3, 1> &position);
        void set_control_effector_velocity(const Eigen::Matrix<double, 3, 1> &velocity);
        void set_control_effector_acceleration(const Eigen::Matrix<double, 3, 1> &acceleration);
    };
}