#pragma once

#include <franka_pole/model.h>
#include <franka_pole/CommandParameters.h>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <mutex>

namespace franka_pole
{
    class ParameterReader;

    ///Parameter set, responsible reading parameters from reader and updating them. It gives quick access to parameters
    class Parameters
    {
    private:
        //References
        std::mutex *_mutex;

        //ROS topics
        bool _publish;
        bool _changed;
        ros::Publisher _publisher;
        ros::Subscriber _subscriber;
        void _receive(const CommandParameters::ConstPtr &msg);
        void _send();

        void _receive_uint(unsigned int *dest, unsigned int source);
        void _receive_double(double *dest, double source);
        void _receive_quaternion(Eigen::Quaterniond *dest, const boost::array<double, 4> &source);
        template<int N> void _receive_vector(Eigen::Matrix<double, N, 1> *dest, const boost::array<double, N> &source);
        static void _send_uint(unsigned int source, unsigned int *dest);
        static void _send_double(double source, double *dest);
        static void _send_quaternion(const Eigen::Quaterniond &source, boost::array<double, 4> *dest);
        template<int N> static void _send_vector(const Eigen::Matrix<double, N, 1> &source, boost::array<double, N> *dest);

    public:
        ///Creates parameter set
        ///@param mutex Reference system-wide mutex
        ///@param reader Parameter reader to read parameters from
        ///@param node_handle ROS node handle
        ///@param publish `true` if the object is responsible for re-publishing the parameters after receiving it
        Parameters(std::mutex *mutex, const ParameterReader &reader, ros::NodeHandle &node_handle, bool publish);
        
        // Essential
        std::string namespacee; ///< Namespace of all ROS topics and parameters
        std::string arm_id;     ///< Arm name
        bool simulated;         ///< `true` if is a Gazebo simulation, `false` if is real hardware
        Model model;            ///< Hardware configuration

        // Periods
        unsigned int franka_period;     ///< Period of franka state update, milliseconds
        unsigned int pole_period;       ///< Period of pole state update, milliseconds
        unsigned int command_period;    ///< Period of franka torque update (and helper contollers), milliseconds
        unsigned int publish_period;    ///< Period between publishing to ROS topics, milliseconds
        unsigned int controller_period; ///< Period of high-level controllers

        // Target state and constraints
        Eigen::Matrix<double, 3, 1> target_effector_position;   ///< Effector's target position
        Eigen::Quaterniond target_effector_orientation;         ///< Effector's target orientation
        double target_joint0_position;                          ///< Robots's target angle of first joint
        bool target_joint0_stuck;                               ///< `true` if first joint is stuck, `false` if first joint is active
        Eigen::Matrix<double, 3, 1> min_effector_position;      ///< Higher boundary for effector position
        Eigen::Matrix<double, 3, 1> max_effector_position;      ///< Lower boundary for effector position
        Eigen::Matrix<double, 3, 1> min_effector_velocity;      ///< Higher boundary for effector velocity
        Eigen::Matrix<double, 3, 1> max_effector_velocity;      ///< Lower boundary for effector velocity

        // Initial state
        Eigen::Matrix<double, 3, 1> initial_effector_position;  ///< Effector's initial position
        Eigen::Quaterniond initial_effector_orientation;        ///< Effector's initial orientation
        double initial_joint0_position;                         ///< Robot's initial angle of first joint
        Eigen::Matrix<double, 2, 1> initial_pole_positions;     ///< Pole's initial joint angles
        Eigen::Matrix<double, 2, 1> initial_pole_velocities;    ///< Pole's initial joint angular velocities

        // Stiffness
        Eigen::Matrix<double, 3, 1> outbound_translation_stiffness; ///< Translational stiffness gain when the effector is out of it's boundaries
        Eigen::Matrix<double, 3, 1> outbound_translation_damping;   ///< Translational damping gain when the effector is out of it's boundaries
        Eigen::Matrix<double, 3, 1> outbound_rotation_stiffness;    ///< Rotational stiffness gain when the effector is out of it's boundaries
        Eigen::Matrix<double, 3, 1> outbound_rotation_damping;      ///< Rotational damping gain when the effector is out of it's boundaries
        Eigen::Matrix<double, 3, 1> translation_stiffness;          ///< Translational stiffness gain when the effector is in it's boundaries
        Eigen::Matrix<double, 3, 1> translation_damping;            ///< Translational damping gain when the effector is in it's boundaries
        Eigen::Matrix<double, 3, 1> rotation_stiffness;             ///< Rotational stiffness gain when the effector is in it's boundaries
        Eigen::Matrix<double, 3, 1> rotation_damping;               ///< Rotational damping gain when the effector is in it's boundaries

        Eigen::Matrix<double, 7, 1> joint_stiffness;                ///< Joint-space stiffness gain
        Eigen::Matrix<double, 7, 1> joint_damping;                  ///< Joint-space damping gain

        Eigen::Matrix<double, 7, 1> nullspace_stiffness;            ///< Nullspace stiffness gain
        Eigen::Matrix<double, 7, 1> nullspace_damping;              ///< Nullspace damping gain

        double dynamics;                                            ///< Multiplier of torque calculated bt inverse dynamics
        bool pure_dynamics;                                         ///< `true` if cartesian control should not be applied as force directly, but passed to inverse dynamics control instead

        // Filters
        double pole_angle_filter;   ///< Filter factor of pole angle. 0.0 for no filter
        double pole_dangle_filter;  ///< Filter factor of pole anglular velocity. 0.0 for no filter

        // Noise
        Eigen::Matrix<double, 7, 1> joint_position_mean;                ///< Mean value of noise added to joint position measurements
        Eigen::Matrix<double, 7, 1> joint_position_standard_deviation;  ///< Standard deviation of noise added to joint position measurements
        Eigen::Matrix<double, 7, 1> joint_velocity_standard_deviation;  ///< Standard deviation of noise added to joint velocity measurements
        Eigen::Matrix<double, 2, 1> pole_angle_mean;                    ///< Mean value of noise added to pole angle measurements
        Eigen::Matrix<double, 2, 1> pole_angle_standard_deviation;      ///< Standard deviation of noise added to pole angle measurements

        // Reset
        double hardware_reset_duration;                         ///< Duration of hardware reset
        Eigen::Matrix<double, 7, 1> hardware_reset_stiffness;   ///< Joint-space stiffness gain during hardware reset
        Eigen::Matrix<double, 7, 1> hardware_reset_damping;     ///< Joint-space damping gain during hardware reset

        // Pole control
        Eigen::Matrix<double, 8, 1> pole_control;   ///< Gain to be multiplied with observation vector

        //Test
        double startup_time;                        ///< Duration till test and simple controllers gradually increase their output from zero to full
        bool test_rectangle;                        ///< `true` if tested on rectangular input, `false` if on sinewave
        Eigen::Matrix<double, 3, 1> test_amplitude; ///< Amplitude of input signal in three axis, meters
        Eigen::Matrix<double, 3, 1> test_frequency; ///< Frequency of input signal in three axis, hertz
        Eigen::Matrix<double, 3, 1> test_phase;     ///< Phase of input signal in three axis, radians
    };
}