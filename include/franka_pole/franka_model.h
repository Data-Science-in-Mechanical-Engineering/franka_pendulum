#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <Eigen/Dense>

namespace franka_pole
{
    class Parameters;

    ///Model of the robot, responsible for kinematics and dynamics calculations
    class FrankaModel
    {
    private:
        //References
        const Parameters *_parameters;

        //Pinocchio technical
        size_t _effector_frame_id;
        size_t _pole_frame_id;
        pinocchio::Model _model;
        pinocchio::Data _data;

    public:
        ///Creates franka model
        ///@param parameters Reference to parameters object
        FrankaModel(const Parameters *parameters);
        ///Computes gravity that is compensated by the Franka Interface
        ///@param joint_positions 7-dimensional vector of joint positions
        ///@return 7-dimensional vector of torque
        Eigen::Matrix<double, 7, 1> get_gravity_compensation(const Eigen::Matrix<double, 7, 1> &joint_positions);
        ///Computes gravity term of the robotic equation (without compensated gravity)
        ///@param joint_positions 7-dimensional vector of joint positions
        ///@return 9-dimensional vector of torque
        Eigen::Matrix<double, 9, 1> get_gravity9(const Eigen::Matrix<double, 7, 1> &joint_positions);
        ///Computes gravity term of the robotic equation (without compensated gravity)
        ///@param joint_positions 7-dimensional vector of joint positions
        ///@param pole_joint_positions 2-dimensional vector of pole joint positions
        ///@return 10-dimensional vector of torque
        Eigen::Matrix<double, 10, 1> get_gravity10(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_positions);
        ///Computes gravity term of the robotic equation (without compensated gravity)
        ///@param joint_positions 7-dimensional vector of joint positions
        ///@param pole_joint_positions 2-dimensional vector of pole joint positions
        ///@return 11-dimensional vector of torque
        Eigen::Matrix<double, 11, 1> get_gravity11(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_positions);
        ///Computes Coriolis term of the robotic equation
        ///@param joint_positions 7-dimensional vector of joint positions
        ///@param joint_velocities 7-dimensional vector of joint velocities
        ///@return 9-dimensional vector of torque
        Eigen::Matrix<double, 9, 1> get_coriolis9(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 7, 1> &joint_velocities);
        ///Computes Coriolis term of the robotic equation
        ///@param joint_positions 7-dimensional vector of joint positions
        ///@param joint_velocities 7-dimensional vector of joint velocities
        ///@param pole_joint_positions 2-dimensional vector of pole joint positions
        ///@param pole_joint_velocities 2-dimensional vector of pole joint velocities
        ///@return 10-dimensional vector of torque
        Eigen::Matrix<double, 10, 1> get_coriolis10(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 7, 1> &joint_velocities, const Eigen::Matrix<double, 2, 1> &pole_joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_velocities);
        ///Computes Coriolis term of the robotic equation
        ///@param joint_positions 7-dimensional vector of joint positions
        ///@param joint_velocities 7-dimensional vector of joint velocities
        ///@param pole_joint_positions 2-dimensional vector of pole joint positions
        ///@param pole_joint_velocities 2-dimensional vector of pole joint velocities
        ///@return 11-dimensional vector of torque
        Eigen::Matrix<double, 11, 1> get_coriolis11(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 7, 1> &joint_velocities, const Eigen::Matrix<double, 2, 1> &pole_joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_velocities);
        ///Computes mass matrix term of the robotic equation
        ///@param joint_positions 7-dimensional vector of joint positions
        ///@return 9x9-dimensional matrix
        Eigen::Matrix<double, 9, 9> get_mass_matrix9(const Eigen::Matrix<double, 7, 1> &joint_positions);
        ///Computes mass matrix term of the robotic equation
        ///@param joint_positions 7-dimensional vector of joint positions
        ///@param pole_joint_positions 2-dimensional vector of pole joint positions
        ///@return 10x10-dimensional matrix
        Eigen::Matrix<double, 10, 10> get_mass_matrix10(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_positions);
        ///Computes mass matrix term of the robotic equation
        ///@param joint_positions 7-dimensional vector of joint positions
        ///@param pole_joint_positions 2-dimensional vector of pole joint positions
        ///@return 11x11-dimensional matrix
        Eigen::Matrix<double, 11, 11> get_mass_matrix11(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_positions);
        ///Computes robot's end effector Jacobian
        ///@param joint_positions 7-dimensional vector of joint positions
        ///@return 6x7-dimensional matrix
        Eigen::Matrix<double, 6, 7> get_effector_jacobian(const Eigen::Matrix<double, 7, 1> &joint_positions);
        ///Computes centroidal acceleration of the robot's end effector
        ///@param joint_positions 7-dimensional vector of joint positions
        ///@param joint_velocities 7-dimensional vector of joint velocities
        ///@return 3-dimensional acceleration vector
        Eigen::Matrix<double, 3, 1> get_effector_centroidal_acceleration(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 7, 1> &joint_velocities);
        ///Computes robot's end effector position and orientation based on joint positions
        ///@param joint_positions 7-dimensional vector of joint positions
        ///@param effector_orientation quaternion to receive orientation
        ///@return 3-dimensional position vector
        Eigen::Matrix<double, 3, 1> effector_forward_kinematics(const Eigen::Matrix<double, 7, 1> &joint_positions, Eigen::Quaterniond *effector_orientation);
        ///Computes pole's position and orientation based on joint positions
        ///@param joint_positions 7-dimensional vector of joint positions
        ///@param pole_joint_positions 2-dimensional vector of pole joint positions
        ///@param pole_orientation quaternion to receive orientation
        ///@return 3-dimensional position vector
        Eigen::Matrix<double, 3, 1> pole_forward_kinematics(const Eigen::Matrix<double, 7, 1> &joint_positions, const Eigen::Matrix<double, 2, 1> &pole_joint_positions, Eigen::Quaterniond *pole_orientation);
        ///Computes robot's joint positions based on effector position and orientation
        ///@param effector_position End effector position
        ///@param effector_orientation End effector orientation
        ///@param joint0 Angle of the first joint (may be `NaN`)
        ///@param hint Initial guess
        ///@return 7-dimensional vector of joint positions
        ///@throw `std::runtime_error` if numer of iteration exeeded (in particular because the desired pose is not reachable)
        Eigen::Matrix<double, 7, 1> effector_inverse_kinematics(const Eigen::Matrix<double, 3, 1> &effector_position, const Eigen::Quaterniond &effector_orientation, double joint0, const Eigen::Matrix<double, 7, 1> &hint);
    };
}