#pragma once

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <geometry_msgs/PoseStamped.h>

#include <memory>
#include <string>
#include <vector>
#include <ros/time.h>

namespace franka_pole
{

class CartesianController : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface, hardware_interface::EffortJointInterface, franka_hw::FrankaStateInterface>
{
 public:
  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  //ROS technical
  std::unique_ptr<franka_hw::FrankaStateHandle> _state_handle;
  std::unique_ptr<franka_hw::FrankaModelHandle> _model_handle;
  std::vector<hardware_interface::JointHandle> _joint_handles;
  ros::Subscriber _cartesian_target_subscriber;

  //Pinocchio technical
  size_t _pinocchio_joint_ids[10];
  pinocchio::Model _pinocchio_model;
  pinocchio::Data _pinocchio_data;

  //Control
  double _nullspace_stiffness; //Initialized in init()
  double _nullspace_damping;
  Eigen::Matrix<double, 6, 6> _cartesian_stiffness;
  Eigen::Matrix<double, 6, 6> _cartesian_damping;
  
  Eigen::Vector3d _position_target;  //Initialized in starting()
  Eigen::Quaterniond _orientation_target;

  Eigen::Matrix<double, 7, 1> _inverse_kinematics(Eigen::Vector3d position, Eigen::Quaterniond prientation);
  template <int H, int W> Eigen::Matrix<double, W, H> _pseudo_inverse(const Eigen::Matrix<double, H, W> &matrix, bool damped);
  void _cartesian_target_callback(const geometry_msgs::PoseStampedConstPtr &msg);
};

}  // namespace franka_pole