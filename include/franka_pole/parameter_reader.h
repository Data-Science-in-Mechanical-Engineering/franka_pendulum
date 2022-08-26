#pragma once

#include <franka_pole/model.h>

#include <ros/node_handle.h>
#include <Eigen/Dense>

namespace franka_pole
{
    ///Parameter reader, responsible for reading parameters from ROS parameters and managing fallbacks
    class ParameterReader
    {
    private:
        std::string _namespace;
        ros::NodeHandle &_node_handle;

        std::string _read_string(const std::string name) const;
        bool _read_bool(const std::string name) const;
        unsigned int _read_uint(const std::string name) const;
        double _read_double(const std::string name) const;
        static double _check_double(const std::string name, double value);
        template<int N> Eigen::Matrix<double, N, 1> _read_vector(const std::string name) const;
        template<int N> static bool _nan_vector(const Eigen::Matrix<double, N, 1> &vector);
        template<int N> static Eigen::Matrix<double, N, 1> _check_vector(const std::string name, const Eigen::Matrix<double, N, 1> &vector);
        template<int N> static Eigen::Matrix<double, N, 1> _replace_vector(const Eigen::Matrix<double, N, 1> &dest, const Eigen::Matrix<double, N, 1> &source);
        Eigen::Quaterniond _read_quaternion(const std::string name) const;
        static bool _nan_quaternion(const Eigen::Quaterniond &quaternion);
        static Eigen::Quaterniond _replace_quaternion(const Eigen::Quaterniond &dest, const Eigen::Quaterniond &source);
        static Eigen::Quaterniond _check_quaternion(const std::string name, const Eigen::Quaterniond &quaternion);

    public:
        ///Creates parameter reader
        ///@param node_handle ROS node handle
        ParameterReader(ros::NodeHandle &node_handle);

        // Essential

        std::string namespacee() const; ///< Namespace of all ROS topics and parameters
        std::string arm_id() const;     ///< Arm name
        bool simulated() const;         ///< `true` if is a Gazebo simulation, `false` if is real hardware
        Model model() const;            ///< Hardware configuration

        // Periods
        unsigned int franka_period() const;     ///< Period of franka state update, milliseconds
        unsigned int pole_period() const;       ///< Period of pole state update, milliseconds
        unsigned int command_period() const;    ///< Period of franka torque update (and helper contollers), milliseconds
        unsigned int publish_period() const;    ///< Period between publishing to ROS topics, milliseconds
        unsigned int controller_period() const; ///< Period of high-level controllers

        // Target state and constraints
        Eigen::Matrix<double, 3, 1> target_effector_position() const;   ///< Effector's target position
        Eigen::Quaterniond target_effector_orientation() const;         ///< Effector's target orientation
        double target_joint0_position() const;                          ///< Effector's target angle of first joint
        bool target_joint0_stuck() const;                               ///< `true` if first joint is stuck, `false` if first joint is active
        Eigen::Matrix<double, 3, 1> min_effector_position() const;      ///< Higher boundary for effector position
        Eigen::Matrix<double, 3, 1> max_effector_position() const;      ///< Lower boundary for effector position
        Eigen::Matrix<double, 3, 1> min_effector_velocity() const;      ///< Higher boundary for effector velocity
        Eigen::Matrix<double, 3, 1> max_effector_velocity() const;      ///< Lower boundary for effector velocity

        // Initial state
        Eigen::Matrix<double, 3, 1> initial_effector_position() const;  ///< Effector's initial position
        Eigen::Quaterniond initial_effector_orientation() const;        ///< Effector's initial orientation
        double initial_joint0_position() const;                         ///< Effector's initial angle of first joint
        Eigen::Matrix<double, 2, 1> initial_pole_positions() const;     ///< Pole's initial joint angles
        Eigen::Matrix<double, 2, 1> initial_pole_velocities() const;    ///< Pole's initial joint angular velocities

        // Stiffness
        Eigen::Matrix<double, 3, 1> outbound_translation_stiffness() const; ///< Translational stiffness gain when the effector is out of it's boundaries
        Eigen::Matrix<double, 3, 1> outbound_translation_damping() const;   ///< Translational damping gain when the effector is out of it's boundaries
        Eigen::Matrix<double, 3, 1> outbound_rotation_stiffness() const;    ///< Rotational stiffness gain when the effector is out of it's boundaries
        Eigen::Matrix<double, 3, 1> outbound_rotation_damping() const;      ///< Rotational damping gain when the effector is out of it's boundaries
        Eigen::Matrix<double, 3, 1> translation_stiffness() const;          ///< Translational stiffness gain when the effector is in it's boundaries
        Eigen::Matrix<double, 3, 1> translation_damping() const;            ///< Translational damping gain when the effector is in it's boundaries
        Eigen::Matrix<double, 3, 1> rotation_stiffness() const;             ///< Rotational stiffness gain when the effector is in it's boundaries
        Eigen::Matrix<double, 3, 1> rotation_damping() const;               ///< Rotational damping gain when the effector is in it's boundaries

        Eigen::Matrix<double, 7, 1> joint_stiffness() const;                ///< Joint-space stiffness gain
        Eigen::Matrix<double, 7, 1> joint_damping() const;                  ///< Joint-space damping gain

        Eigen::Matrix<double, 7, 1> nullspace_stiffness() const;            ///< Nullspace stiffness gain
        Eigen::Matrix<double, 7, 1> nullspace_damping() const;              ///< Nullspace damping gain

        double dynamics() const;                                            ///< Multiplier of torque calculated bt inverse dynamics
        bool pure_dynamics() const;                                         ///< `true` if cartesian control should not be applied as force directly, but passed to inverse dynamics control instead

        // Filters
        double pole_angle_filter() const;   ///< Filter factor of pole angle. 0.0 for no filter
        double pole_dangle_filter() const;  ///< Filter factor of pole anglular velocity. 0.0 for no filter

        // Noise
        Eigen::Matrix<double, 7, 1> joint_position_mean() const;                ///< Mean value of noise added to joint position measurements
        Eigen::Matrix<double, 7, 1> joint_position_standard_deviation() const;  ///< Standard deviation of noise added to joint position measurements
        Eigen::Matrix<double, 7, 1> joint_velocity_standard_deviation() const;  ///< Standard deviation of noise added to joint velocity measurements
        Eigen::Matrix<double, 2, 1> pole_angle_mean() const;                    ///< Mean value of noise added to pole angle measurements
        Eigen::Matrix<double, 2, 1> pole_angle_standard_deviation() const;      ///< Standard deviation of noise added to pole angle measurements
        
        // Reset
        double hardware_reset_duration() const;                         ///< Duration of hardware reset
        Eigen::Matrix<double, 7, 1> hardware_reset_stiffness() const;   ///< Joint-space stiffness gain during hardware reset
        Eigen::Matrix<double, 7, 1> hardware_reset_damping() const;     ///< Joint-space damping gain during hardware reset

        // Pole control
        Eigen::Matrix<double, 8, 1> pole_control() const;    ///< Gain to be multiplied with observation vector

        //Test
        double startup_time() const;                        ///< Duration till test and simple controllers gradually increase their output from zero to full
        bool test_rectangle() const;                        ///< `true` if tested on rectangular input, `false` if on sinewave
        Eigen::Matrix<double, 3, 1> test_amplitude() const; ///< Amplitude of input signal in three axis, meters
        Eigen::Matrix<double, 3, 1> test_frequency() const; ///< Frequency of input signal in three axis, hertz
        Eigen::Matrix<double, 3, 1> test_phase() const;     ///< Phase of input signal in three axis, radians
    };
}