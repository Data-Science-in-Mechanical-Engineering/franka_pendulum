#include <franka_pole/parameters.h>

std::string franka_pole::Parameters::_read_string(const std::string name) const
{
    std::string s;
    if (!_node_handle.getParam("/franka_pole/" + name, s)) throw std::runtime_error("franka_pole::Parameters: Could not read parameter " + name);
    return s;
}

bool franka_pole::Parameters::_read_bool(const std::string name) const
{
    bool b;
    if (!_node_handle.getParam("/franka_pole/" + name, b)) throw std::runtime_error("franka_pole::Parameters: Could not read parameter " + name);
    return b;
}

unsigned int franka_pole::Parameters::_read_uint(const std::string name) const
{
    int u;
    if (!_node_handle.getParam("/franka_pole/" + name, u)) throw std::runtime_error("franka_pole::Parameters: Could not read parameter " + name);
    return u;
}

double franka_pole::Parameters::_read_double(const std::string name) const
{
    double d;
    if (!_node_handle.getParam("/franka_pole/" + name, d)) throw std::runtime_error("franka_pole::Parameters: Could not read parameter " + name);
    return d;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> franka_pole::Parameters::_read_vector(const std::string name, size_t dimension) const
{
    std::vector<double> v;
    if (!_node_handle.getParam("/franka_pole/" + name, v)) throw std::runtime_error("franka_pole::Parameters: Could not read parameter " + name);
    if (v.size() != dimension) throw std::runtime_error("franka_pole::Parameters: Parameter " + name + " has invalid dimension");
    return Eigen::Matrix<double, Eigen::Dynamic, 1>::Map(v.data(), dimension, 1);
}

Eigen::Quaterniond franka_pole::Parameters::_read_quaternion(const std::string name) const
{
    std::map<std::string, double> q;
    if (!_node_handle.getParam("/franka_pole/" + name, q)) throw std::runtime_error("franka_pole::Parameters: Could not read parameter " + name);
    if (q.size() != 4 || q.count("w") != 1 || q.count("x") != 1 || q.count("y") != 1 || q.count("z") != 1) throw std::runtime_error("franka_pole::Parameters: Parameter " + name + " is invalid");
    return Eigen::Quaterniond(q["w"], q["x"], q["y"], q["z"]);
}

franka_pole::Parameters::Parameters(ros::NodeHandle &node_handle) : _node_handle(node_handle)
{
}

franka_pole::Model franka_pole::Parameters::model() const
{
    std::string s;
    if (!_node_handle.getParam("/franka_pole/model", s)) throw std::runtime_error("franka_pole::Parameters: Could not read parameter model");
    if (s == "1D") return Model::D1;
    else if (s == "2D") return Model::D2;
    else if (s == "2Db") return Model::D2b;
    else throw std::runtime_error("franka_pole::Parameters: Invalid parameter model");
}

std::string franka_pole::Parameters::arm_id() const { return _read_string("arm_id"); }
bool franka_pole::Parameters::simulated() const { return _read_bool("simulated"); }

unsigned int franka_pole::Parameters::franka_period() const { return _read_uint("franka_period"); }
unsigned int franka_pole::Parameters::pole_period() const { return _read_uint("pole_period"); }

Eigen::Matrix<double, 3, 1> franka_pole::Parameters::target_effector_position() const { return _read_vector("target_effector_position", 3); }
Eigen::Quaterniond franka_pole::Parameters::target_effector_orientation() const { return _read_quaternion("target_effector_orientation"); }
Eigen::Matrix<double, 3, 1> franka_pole::Parameters::min_effector_position() const { return _read_vector("min_effector_position", 3); }
Eigen::Matrix<double, 3, 1> franka_pole::Parameters::max_effector_position() const { return _read_vector("max_effector_position", 3); }

Eigen::Matrix<double, 3, 1> franka_pole::Parameters::initial_effector_position() const { return _read_vector("initial_effector_position", 3); }
Eigen::Quaterniond franka_pole::Parameters::initial_effector_orientation() const { return _read_quaternion("initial_effector_orientation"); }
double franka_pole::Parameters::initial_joint0_position() const { return _read_double("initial_joint0_position"); }
Eigen::Matrix<double, 2, 1> franka_pole::Parameters::initial_pole_positions() const { return _read_vector("initial_pole_positions", 2); }
Eigen::Matrix<double, 2, 1> franka_pole::Parameters::initial_pole_velocities() const { return _read_vector("initial_pole_velocities", 2); }

Eigen::Matrix<double, 3, 1> franka_pole::Parameters::translation_stiffness() const { return _read_vector("translation_stiffness", 3); }
Eigen::Matrix<double, 3, 1> franka_pole::Parameters::translation_stiffness_safety() const { return _read_vector("translation_stiffness_safety", 3); }
Eigen::Matrix<double, 3, 1> franka_pole::Parameters::rotation_stiffness() const { return _read_vector("rotation_stiffness", 3); }
Eigen::Matrix<double, 7, 1> franka_pole::Parameters::nullspace_stiffness() const { return _read_vector("nullspace_stiffness", 7); }
Eigen::Matrix<double, 7, 1> franka_pole::Parameters::joint_stiffness() const { return _read_vector("joint_stiffness", 7); }

Eigen::Matrix<double, 7, 1> franka_pole::Parameters::joint_position_standard_deviation() const { return _read_vector("joint_position_standard_deviation", 7); }
Eigen::Matrix<double, 7, 1> franka_pole::Parameters::joint_velocity_standard_deviation() const { return _read_vector("joint_velocity_standard_deviation", 7); }
double franka_pole::Parameters::pole_period_standard_deviation() const { return _read_double("pole_period_standard_deviation"); }
Eigen::Matrix<double, 2, 1> franka_pole::Parameters::pole_angle_standard_deviation() const { return _read_vector("pole_angle_standard_deviation", 2); }