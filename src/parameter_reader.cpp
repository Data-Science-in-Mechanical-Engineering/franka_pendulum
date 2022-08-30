#include <franka_pole/parameter_reader.h>

std::string franka_pole::ParameterReader::_read_string(const std::string name) const
{
    std::string s;
    if (!_node_handle.getParam("/" + _namespace + "/" + name, s)) throw std::runtime_error("franka_pole::ParameterReader: Could not read parameter " + name);
    return s;
}

bool franka_pole::ParameterReader::_read_bool(const std::string name) const
{
    bool b;
    if (!_node_handle.getParam("/" + _namespace + "/" + name, b)) throw std::runtime_error("franka_pole::ParameterReader: Could not read parameter " + name);
    return b;
}

unsigned int franka_pole::ParameterReader::_read_uint(const std::string name) const
{
    int u;
    if (!_node_handle.getParam("/" + _namespace + "/" + name, u)) throw std::runtime_error("franka_pole::ParameterReader: Could not read parameter " + name);
    return u;
}

double franka_pole::ParameterReader::_read_double(const std::string name) const
{
    double d;
    if (!_node_handle.getParam("/" + _namespace + "/" + name, d)) throw std::runtime_error("franka_pole::ParameterReader: Could not read parameter " + name);
    return d;
}

double franka_pole::ParameterReader::_check_double(const std::string name, double value)
{
    if (isnan(value)) throw std::runtime_error("franka_pole::ParameterReader: parameter " + name + " is NaN");
    return value;
}

template<int N> Eigen::Matrix<double, N, 1> franka_pole::ParameterReader::_read_vector(const std::string name) const
{
    std::vector<double> v;
    if (!_node_handle.getParam("/" + _namespace + "/" + name, v)) throw std::runtime_error("franka_pole::ParameterReader: Could not read parameter " + name);
    if (v.size() != N) throw std::runtime_error("franka_pole::ParameterReader: Parameter " + name + " has invalid dimension");
    return Eigen::Matrix<double, N, 1>::Map(v.data());
}

template<int N> bool franka_pole::ParameterReader::_nan_vector(const Eigen::Matrix<double, N, 1> &vector)
{
    for (size_t i = 0; i < N; i++)
    {
        if (isnan(vector(i))) return true;
    }
    return false;
}

template<int N> Eigen::Matrix<double, N, 1> franka_pole::ParameterReader::_replace_vector(const Eigen::Matrix<double, N, 1> &dest, const Eigen::Matrix<double, N, 1> &source)
{
    Eigen::Matrix<double, N, 1> vector = dest;
    for (size_t i = 0; i < N; i++)
    {
        if (isnan(vector(i))) vector(i) = source(i);
    }
    return vector;
}

template<int N> Eigen::Matrix<double, N, 1> franka_pole::ParameterReader::_check_vector(const std::string name, const Eigen::Matrix<double, N, 1> &vector)
{
    if (_nan_vector(vector)) throw std::runtime_error("franka_pole::ParameterReader: parameter " + name + " contains NaN values");
    return vector;
}

Eigen::Quaterniond franka_pole::ParameterReader::_read_quaternion(const std::string name) const
{
    std::map<std::string, double> q;
    if (!_node_handle.getParam("/" + _namespace + "/" + name, q)) throw std::runtime_error("franka_pole::ParameterReader: Could not read parameter " + name);
    if (q.size() != 4 || q.count("w") != 1 || q.count("x") != 1 || q.count("y") != 1 || q.count("z") != 1) throw std::runtime_error("franka_pole::ParameterReader: Parameter " + name + " is invalid");
    return Eigen::Quaterniond(q["w"], q["x"], q["y"], q["z"]);
}

bool franka_pole::ParameterReader::_nan_quaternion(const Eigen::Quaterniond &quaternion)
{
    return isnan(quaternion.w()) || isnan(quaternion.x()) || isnan(quaternion.y()) || isnan(quaternion.z());
}

Eigen::Quaterniond franka_pole::ParameterReader::_replace_quaternion(const Eigen::Quaterniond &dest, const Eigen::Quaterniond &source)
{
    Eigen::Quaterniond quaternion = dest;
    if (isnan(quaternion.w())) quaternion.w() = source.w();
    if (isnan(quaternion.x())) quaternion.x() = source.x();
    if (isnan(quaternion.y())) quaternion.y() = source.y();
    if (isnan(quaternion.z())) quaternion.z() = source.z();
    return quaternion;
}

Eigen::Quaterniond franka_pole::ParameterReader::_check_quaternion(const std::string name, const Eigen::Quaterniond &quaternion)
{
    if (_nan_quaternion(quaternion)) throw std::runtime_error("franka_pole::ParameterReader: parameter " + name + " contains NaN values");
    return quaternion;
}

franka_pole::ParameterReader::ParameterReader(ros::NodeHandle &node_handle) : _node_handle(node_handle)
{
    if (node_handle.getNamespace().find("controller") != std::string::npos)
    {
        if (!_node_handle.getParam("namespace", _namespace)) throw std::runtime_error("franka_pole::ParameterReader::ParameterReader(): Could not read parameter namespace");
    }
    else
    {
        if (!_node_handle.getParam("/gazebo/namespace", _namespace)) throw std::runtime_error("franka_pole::ParameterReader::ParameterReader(): Could not read parameter namespace");
    }
}

std::string franka_pole::ParameterReader::namespacee() const { return _namespace; }
std::string franka_pole::ParameterReader::arm_id() const { return _read_string("arm_id"); }
bool franka_pole::ParameterReader::simulated() const { return _read_bool("simulated"); }
franka_pole::Model franka_pole::ParameterReader::model() const
{
    std::string s;
    if (!_node_handle.getParam("/" + _namespace + "/model", s)) throw std::runtime_error("franka_pole::ParameterReader: Could not read parameter model");
    if (s == "0D") return Model::D0;
    else if (s == "1D") return Model::D1;
    else if (s == "2D") return Model::D2;
    else if (s == "2Db") return Model::D2b;
    else if (s == "2Dc") return Model::D2c;
    else throw std::runtime_error("franka_pole::ParameterReader: Invalid parameter model");
}

unsigned int franka_pole::ParameterReader::franka_period() const { return _read_uint("franka_period"); }
unsigned int franka_pole::ParameterReader::pole_period() const { return _read_uint("pole_period"); }
unsigned int franka_pole::ParameterReader::command_period() const { return _read_uint("command_period"); }
unsigned int franka_pole::ParameterReader::publish_period() const { return _read_uint("publish_period"); }
unsigned int franka_pole::ParameterReader::controller_period() const { return _read_uint("controller_period"); }

Eigen::Matrix<double, 3, 1> franka_pole::ParameterReader::target_effector_position() const { return _check_vector("target_effector_position", _read_vector<3>("target_effector_position")); }
Eigen::Quaterniond franka_pole::ParameterReader::target_effector_orientation() const { return _check_quaternion("target_effector_orientation", _read_quaternion("target_effector_orientation")); }
Eigen::Matrix<double, 7, 1> franka_pole::ParameterReader::target_joint_weights() const { return _check_vector("target_joint_weights", _read_vector<7>("target_joint_weights")); }
Eigen::Matrix<double, 3, 1> franka_pole::ParameterReader::min_effector_position() const { return _check_vector("min_effector_position", _read_vector<3>("min_effector_position")); }
Eigen::Matrix<double, 3, 1> franka_pole::ParameterReader::max_effector_position() const { return _check_vector("max_effector_position", _read_vector<3>("max_effector_position")); }
Eigen::Matrix<double, 3, 1> franka_pole::ParameterReader::min_effector_velocity() const { return _check_vector("min_effector_velocity", _read_vector<3>("min_effector_velocity")); }
Eigen::Matrix<double, 3, 1> franka_pole::ParameterReader::max_effector_velocity() const { return _check_vector("max_effector_velocity", _read_vector<3>("max_effector_velocity")); }

Eigen::Matrix<double, 3, 1> franka_pole::ParameterReader::initial_effector_position() const { auto v = _read_vector<3>("initial_effector_position"); if (_nan_vector(v)) v = _replace_vector(v, target_effector_position()); return _check_vector("initial_effector_position", v); }
Eigen::Quaterniond franka_pole::ParameterReader::initial_effector_orientation() const { auto q = _read_quaternion("initial_effector_orientation"); if (_nan_quaternion(q)) q = _replace_quaternion(q, target_effector_orientation()); return _check_quaternion("initial_effector_orientation", q); }
Eigen::Matrix<double, 7, 1> franka_pole::ParameterReader::initial_joint_weights() const { auto v = _read_vector<7>("initial_joint_weights"); if (_nan_vector(v)) v = _replace_vector<7>(v, target_joint_weights().array().sqrt().matrix()); return _check_vector("initial_joint_weights", v); }
Eigen::Matrix<double, 2, 1> franka_pole::ParameterReader::initial_pole_positions() const { return _check_vector("initial_pole_positions", _read_vector<2>("initial_pole_positions")); }
Eigen::Matrix<double, 2, 1> franka_pole::ParameterReader::initial_pole_velocities() const { return _check_vector("initial_pole_velocities", _read_vector<2>("initial_pole_velocities")); }

Eigen::Matrix<double, 3, 1> franka_pole::ParameterReader::outbound_translation_stiffness() const { return _check_vector("outbound_translation_stiffness", _read_vector<3>("outbound_translation_stiffness")); }
Eigen::Matrix<double, 3, 1> franka_pole::ParameterReader::outbound_translation_damping() const { auto v = _read_vector<3>("outbound_translation_damping"); if (_nan_vector(v)) v = _replace_vector<3>(v, 2*outbound_translation_stiffness().array().sqrt().matrix()); return _check_vector("outbound_translation_damping", v); }
Eigen::Matrix<double, 3, 1> franka_pole::ParameterReader::outbound_rotation_stiffness() const { return _check_vector("outbound_rotation_stiffness", _read_vector<3>("outbound_rotation_stiffness")); }
Eigen::Matrix<double, 3, 1> franka_pole::ParameterReader::outbound_rotation_damping() const { auto v = _read_vector<3>("outbound_rotation_damping"); if (_nan_vector(v)) v = _replace_vector<3>(v, 2*outbound_rotation_stiffness().array().sqrt().matrix()); return _check_vector("outbound_rotation_damping", v); }
Eigen::Matrix<double, 3, 1> franka_pole::ParameterReader::translation_stiffness() const { auto v = _read_vector<3>("translation_stiffness"); if (_nan_vector(v)) v = _replace_vector<3>(v, outbound_translation_stiffness()); return _check_vector("translation_stiffness", v); }
Eigen::Matrix<double, 3, 1> franka_pole::ParameterReader::translation_damping() const { auto v = _read_vector<3>("translation_damping"); if (_nan_vector(v)) v = _replace_vector<3>(v, 2*translation_stiffness().array().sqrt().matrix()); return _check_vector("translation_damping", v); }
Eigen::Matrix<double, 3, 1> franka_pole::ParameterReader::rotation_stiffness() const { auto v = _read_vector<3>("rotation_stiffness"); if (_nan_vector(v)) v = _replace_vector<3>(v, outbound_rotation_stiffness()); return _check_vector("rotation_stiffness", v); }
Eigen::Matrix<double, 3, 1> franka_pole::ParameterReader::rotation_damping() const { auto v = _read_vector<3>("rotation_damping"); if (_nan_vector(v)) v = _replace_vector<3>(v, 2*rotation_stiffness().array().sqrt().matrix()); return _check_vector("rotation_damping", v); }

Eigen::Matrix<double, 7, 1> franka_pole::ParameterReader::joint_stiffness() const { return _check_vector("joint_stiffness", _read_vector<7>("joint_stiffness")); }
Eigen::Matrix<double, 7, 1> franka_pole::ParameterReader::joint_damping() const { auto v = _read_vector<7>("joint_damping"); if (_nan_vector(v)) v = _replace_vector<7>(v, 2*joint_stiffness().array().sqrt().matrix()); return _check_vector("joint_damping", v); }

Eigen::Matrix<double, 7, 1> franka_pole::ParameterReader::nullspace_stiffness() const { return _check_vector("nullspace_stiffness", _read_vector<7>("nullspace_stiffness")); }
Eigen::Matrix<double, 7, 1> franka_pole::ParameterReader::nullspace_damping() const { auto v = _read_vector<7>("nullspace_damping"); if (_nan_vector(v)) v = _replace_vector<7>(v, 2*nullspace_stiffness().array().sqrt().matrix()); return _check_vector("nullspace_damping", v); }

double franka_pole::ParameterReader::dynamics() const { return _check_double("dynamics", _read_double("dynamics")); }
bool franka_pole::ParameterReader::pure_dynamics() const { return _read_bool("pure_dynamics"); }

double franka_pole::ParameterReader::pole_angle_filter() const { return _check_double("pole_angle_filter", _read_double("pole_angle_filter")); }
double franka_pole::ParameterReader::pole_dangle_filter() const { double d = _read_double("pole_angle_filter"); if (isnan(d)) d = pole_angle_filter(); return _check_double("pole_angle_filter", d); }

Eigen::Matrix<double, 7, 1> franka_pole::ParameterReader::joint_position_mean() const { return _check_vector("joint_position_mean", _read_vector<7>("joint_position_mean")); }
Eigen::Matrix<double, 7, 1> franka_pole::ParameterReader::joint_position_standard_deviation() const { return _check_vector("joint_position_standard_deviation", _read_vector<7>("joint_position_standard_deviation")); }
Eigen::Matrix<double, 7, 1> franka_pole::ParameterReader::joint_velocity_standard_deviation() const { auto v = _read_vector<7>("joint_velocity_standard_deviation"); if (_nan_vector(v)) v = _replace_vector<7>(v, joint_position_standard_deviation()); return _check_vector("joint_velocity_standard_deviation", v); }
Eigen::Matrix<double, 2, 1> franka_pole::ParameterReader::pole_angle_mean() const { return _check_vector("pole_angle_mean", _read_vector<2>("pole_angle_mean")); }
Eigen::Matrix<double, 2, 1> franka_pole::ParameterReader::pole_angle_standard_deviation() const { return _check_vector("pole_angle_standard_deviation", _read_vector<2>("pole_angle_standard_deviation")); }

double franka_pole::ParameterReader::hardware_reset_duration() const { return _check_double("hardware_reset_durtion", _read_double("hardware_reset_durtion")); }
Eigen::Matrix<double, 7, 1> franka_pole::ParameterReader::hardware_reset_stiffness() const { return _check_vector("hardware_reset_stiffness", _read_vector<7>("hardware_reset_stiffness")); }
Eigen::Matrix<double, 7, 1> franka_pole::ParameterReader::hardware_reset_damping() const { auto v = _read_vector<7>("hardware_reset_damping"); if (_nan_vector(v)) v = _replace_vector<7>(v, 2*hardware_reset_stiffness().array().sqrt().matrix()); return _check_vector("hardware_reset_damping", v); }

Eigen::Matrix<double, 8, 1> franka_pole::ParameterReader::pole_control() const { return _check_vector("pole_control", _read_vector<8>("pole_control")); }

double franka_pole::ParameterReader::startup_time() const { return _check_double("startup_time", _read_double("startup_time")); }
bool franka_pole::ParameterReader::test_rectangle() const { return _read_bool("test_rectangle"); }
Eigen::Matrix<double, 3, 1> franka_pole::ParameterReader::test_amplitude() const { return _check_vector("test_amplitude", _read_vector<3>("test_amplitude")); }
Eigen::Matrix<double, 3, 1> franka_pole::ParameterReader::test_frequency() const { return _check_vector("test_frequency", _read_vector<3>("test_frequency")); }
Eigen::Matrix<double, 3, 1> franka_pole::ParameterReader::test_phase() const { return _check_vector("test_phase", _read_vector<3>("test_phase")); }