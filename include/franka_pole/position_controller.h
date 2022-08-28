#pragma once

#include <franka_pole/controller.h>

#include <Eigen/Dense>

namespace franka_pole
{
    class Parameters;

    ///Position controller is a mid-level helper controller, responsible for getting position and velocity target from higher level controller and returning torque to low-level controller
    class PositionController : public Controller
    {
    private:
        //Time
        unsigned int _controller_period_counter;    ///< _acceleration_target is updates when it reaches "controller_period"

        //Jacobians
        Eigen::Matrix<double, 6, 7> _jacobian;                  ///< Jacobian of the end effector
        Eigen::Matrix<double, 7, 6> _jacobian_transpose;        ///< Transposed Jacobian
        Eigen::Matrix<double, 7, 6> _jacobian_inverse;          ///< Pseudo-invese of Jacobian
        Eigen::Matrix<double, 6, 7> _jacobian_transpose_inverse;///< Pseudo-invese of transposed Jacobian

        //Control
        Eigen::Matrix<double, 6, 1> _velocity_target;               ///< Integrated and constrained input acceleration
        Eigen::Matrix<double, 3, 1> _position_target;               ///< Integrated and constrained input velocity
        Eigen::Matrix<double, 7, 1> _joint_velocities_target;       ///< Target joint velocities, computed from _velocity_target
        Eigen::Matrix<double, 7, 1> _joint_positions_target;        ///< Target joint positions, computed with inverse kinematics
        Eigen::Matrix<double, 7, 1> _initial_joint_positions_target;///< Target joint positions (calculated only on initialization)
        Eigen::Matrix<double, 7, 1> _torque;                        ///< Output torque

        //Additional parameters
        void _update_cartesian_targets(const ros::Time &time, const ros::Duration &period); ///< Performs integration and applies security constraints. Affects _acceleration_target, _velocity_target, _position_target
        void _init_step();                                                                  ///< Sets in-step accumulative variables to their initial values
        void _compute_jacobians();                                                          ///< Computes Jacobains. Affects _jacobian, _jacobian_transpose, _jacobian_inverse, _jacobian_transpose_inverse
        void _cartesian_control();                                                          ///< Applies cartesian controller. Affects _torque (normal) or _control_acceleration (pure acceleration)
        void _joint_space_control();                                                        ///< Applies joint-space controller. Affects _torque (normal) or _control_joint_acceleration (pure acceleration)
        void _nullspace_control();                                                          ///< Applies nullspace controller. Affects _torque (normal) or _control_joint_acceleration (pure acceleration)
        void _gravity_coriolis_control();                                                   ///< Adds gravity and Coriolis terms. Affects _torque

        //Overrides from franka_pole::Controller
        bool _init_level1(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        #ifdef FRANKA_POLE_VELOCITY_INTERFACE
            Eigen::Matrix<double, 6, 1> _get_velocity_level1(const ros::Time &time, const ros::Duration &period) override;
        #else
            Eigen::Matrix<double, 7, 1> _get_torque_level1(const ros::Time &time, const ros::Duration &period) override;
        #endif

    protected:
        //Interface for higher level controllers
        ///Initializes higher level controllers
        ///@param robot_hw `hardware_interface::RobotHW` object
        ///@param node_handle ROS node handle
        ///@return `true` if initialization was successfull
        virtual bool _init_level2(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) = 0;
        ///Gets position command from higher level controllers
        ///@param time Current time
        ///@param period Time from previous update
        ///@return postiion target
        virtual Eigen::Matrix<double, 3, 1> _get_position_level2(const ros::Time &time, const ros::Duration &period) = 0;
        ///Gets velocity command from higher level controllers
        ///@param time Current time
        ///@param period Time from previous update
        ///@return velocity target
        virtual Eigen::Matrix<double, 3, 1> _get_velocity_level2(const ros::Time &time, const ros::Duration &period) = 0;
    };
}