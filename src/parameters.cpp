#include <franka_pole/parameters.h>

franka_pole::Parameters::Parameters(Controller *controller, hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
{
    //Reading parameters
    if (!node_handle.getParam("/franka_pole/arm_id", _arm_id)) throw std::runtime_error("franka_pole::Parameters: Could not read parameter arm_id");
    if (!node_handle.getParam("/franka_pole/simulated", _simulated)) throw std::runtime_error("franka_pole::Parameters: Could not read parameter simulated");
    if (!node_handle.getParam("/franka_pole/two_dimensional", _two_dimensional)) throw std::runtime_error("franka_pole::Parameters: Could not read parameter two_dimensional");
    
    std::vector<double> translation_stiffness;
    if (!node_handle.getParam("/franka_pole/translation_stiffness", translation_stiffness)) throw std::runtime_error("franka_pole::Parameters: Could not read parameter translation_stiffness");
    if (translation_stiffness.size() != 3) throw std::runtime_error("franka_pole::Parameters: Parameter translation_stiffness has invalid dimension");
    _translation_stiffness = Eigen::Matrix<double, 3, 1>::Map(&translation_stiffness[0]);
    
    std::vector<double> rotation_stiffness;
    if (!node_handle.getParam("/franka_pole/rotation_stiffness", rotation_stiffness)) throw std::runtime_error("franka_pole::Parameters: Could not read parameter rotation_stiffness");
    if (rotation_stiffness.size() != 3) throw std::runtime_error("franka_pole::Parameters: Parameter rotation_stiffness has invalid dimension");
    _rotation_stiffness = Eigen::Matrix<double, 3, 1>::Map(&rotation_stiffness[0]);
    
    std::vector<double> nullspace_stiffness;
    if (!node_handle.getParam("/franka_pole/nullspace_stiffness", nullspace_stiffness)) throw std::runtime_error("franka_pole::Parameters: Could not read parameter nullspace_stiffness");
    if (nullspace_stiffness.size() != 7) throw std::runtime_error("franka_pole::Parameters: Parameter nullspace_stiffness has invalid dimension");
    _nullspace_stiffness = Eigen::Matrix<double, 7, 1>::Map(&nullspace_stiffness[0]);
    
    std::vector<double> joint_stiffness;
    if (!node_handle.getParam("/franka_pole/joint_stiffness", joint_stiffness)) throw std::runtime_error("franka_pole::Parameters: Could not read parameter joint_stiffness");
    if (joint_stiffness.size() != 7) throw std::runtime_error("franka_pole::Parameters: Parameter joint_stiffness has invalid dimension");
    _joint_stiffness = Eigen::Matrix<double, 7, 1>::Map(&joint_stiffness[0]);
    
    std::vector<double> box_center;
    if (!node_handle.getParam("/franka_pole/box_center", box_center)) throw std::runtime_error("franka_pole::Parameters: Could not read parameter box_center");
    if (box_center.size() != 3) throw std::runtime_error("franka_pole::Parameters: Parameter box_center has invalid dimension");
    _box_center = Eigen::Matrix<double, 3, 1>::Map(&box_center[0]);
    
    std::vector<double> box_min;
    if (!node_handle.getParam("/franka_pole/box_min", box_min)) throw std::runtime_error("franka_pole::Parameters: Could not read parameter box_min");
    if (box_min.size() != 3) throw std::runtime_error("franka_pole::Parameters: Parameter box_min has invalid dimension");
    _box_min = Eigen::Matrix<double, 3, 1>::Map(&box_min[0]);
    
    std::vector<double> box_max;
    if (!node_handle.getParam("/franka_pole/box_max", box_max)) throw std::runtime_error("franka_pole::Parameters: Could not read parameter box_max");
    if (box_max.size() != 3) throw std::runtime_error("franka_pole::Parameters: Parameter box_max has invalid dimension");
    _box_max = Eigen::Matrix<double, 3, 1>::Map(&box_max[0]);
    
    std::vector<double> initial_joint_positions;
    if (!node_handle.getParam("/franka_pole/initial_joint_positions", initial_joint_positions)) throw std::runtime_error("franka_pole::Parameters: Could not read parameter initial_joint_positions");
    if (initial_joint_positions.size() != 7) throw std::runtime_error("franka_pole::Parameters: Parameter initial_joint_positions has invalid dimension");
    _initial_joint_positions = Eigen::Matrix<double, 7, 1>::Map(&initial_joint_positions[0]);
    
    std::vector<double> initial_pole_positions;
    if (!node_handle.getParam("/franka_pole/initial_pole_positions", initial_pole_positions)) throw std::runtime_error("franka_pole::Parameters: Could not read parameter initial_pole_positions");
    if (initial_pole_positions.size() != 2) throw std::runtime_error("franka_pole::Parameters: Parameter initial_pole_positions has invalid dimension");
    _initial_pole_positions = Eigen::Matrix<double, 2, 1>::Map(&initial_pole_positions[0]);

    _ok = true;
}

bool franka_pole::Parameters::ok() const
{
    return _ok;
}

std::string franka_pole::Parameters::arm_id() const
{
    return _arm_id;
}

bool franka_pole::Parameters::simulated() const
{
    return _simulated;
}

bool franka_pole::Parameters::two_dimensional() const
{
    return _two_dimensional;
}

Eigen::Matrix<double, 3, 1> franka_pole::Parameters::translation_stiffness() const
{
    return _translation_stiffness;
}

Eigen::Matrix<double, 3, 1> franka_pole::Parameters::rotation_stiffness() const
{
    return _rotation_stiffness;
}

Eigen::Matrix<double, 7, 1> franka_pole::Parameters::nullspace_stiffness() const
{
    return _nullspace_stiffness;
}

Eigen::Matrix<double, 7, 1> franka_pole::Parameters::joint_stiffness() const
{
    return _joint_stiffness;
}

Eigen::Matrix<double, 3, 1> franka_pole::Parameters::box_center() const
{
    return _box_center;
}

Eigen::Matrix<double, 3, 1> franka_pole::Parameters::box_min() const
{
    return _box_min;
}

Eigen::Matrix<double, 3, 1> franka_pole::Parameters::box_max() const
{
    return _box_max;
}

Eigen::Matrix<double, 7, 1> franka_pole::Parameters::initial_joint_positions() const
{
    return _initial_joint_positions;
}

Eigen::Matrix<double, 2, 1> franka_pole::Parameters::initial_pole_positions() const
{
    return _initial_pole_positions;
}