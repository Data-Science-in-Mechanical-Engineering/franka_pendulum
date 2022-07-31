#include <franka_pole/parameters.h>
#include <franka_pole/parameter_reader.h>

void franka_pole::Parameters::_callback(const CommandParameters::ConstPtr &msg)
{
    // Periods
    _replace_uint(&franka_period, msg->franka_period);
    _replace_uint(&pole_period, msg->pole_period);
    _replace_uint(&command_period, msg->command_period);
    _replace_uint(&publish_period, msg->publish_period);
    _replace_uint(&controller_period, msg->controller_period);

    // Target state and constraints
    _replace_vector<3>(&target_effector_position, msg->target_effector_position);
    _replace_quaternion(&target_effector_orientation, msg->target_effector_orientation);
    _replace_double(&target_joint0_position, msg->target_joint0_position);
    _replace_vector<3>(&min_effector_position, msg->min_effector_position);
    _replace_vector<3>(&max_effector_position, msg->max_effector_position);

    // Initial state
    _replace_vector<3>(&initial_effector_position, msg->initial_effector_position);
    _replace_quaternion(&initial_effector_orientation, msg->initial_effector_orientation);
    _replace_double(&initial_joint0_position, msg->initial_joint0_position);
    _replace_vector<2>(&initial_pole_positions, msg->initial_pole_positions);
    _replace_vector<2>(&initial_pole_velocities, msg->initial_pole_velocities);

    // Stiffness
    _replace_vector<3>(&outbound_translation_stiffness, msg->outbound_translation_stiffness);
    _replace_vector<3>(&outbound_translation_damping, msg->outbound_translation_damping);
    _replace_vector<3>(&outbound_rotation_stiffness, msg->outbound_rotation_stiffness);
    _replace_vector<3>(&outbound_rotation_damping, msg->outbound_rotation_damping);
    _replace_vector<3>(&translation_stiffness, msg->translation_stiffness);
    _replace_vector<3>(&translation_damping, msg->translation_damping);
    _replace_vector<3>(&rotation_stiffness, msg->rotation_stiffness);
    _replace_vector<3>(&rotation_damping, msg->rotation_damping);

    _replace_vector<7>(&joint_stiffness, msg->joint_stiffness);
    _replace_vector<7>(&joint_damping, msg->joint_damping);

    _replace_vector<7>(&nullspace_stiffness, msg->nullspace_stiffness);
    _replace_vector<7>(&nullspace_damping, msg->nullspace_damping);

    _replace_double(&dynamics, msg->dynamics);

    // Filters
    _replace_double(&pole_angle_filter, msg->pole_angle_filter);
    _replace_double(&pole_dangle_filter, msg->pole_dangle_filter);

    // Noise
    _replace_vector<7>(&joint_position_standard_deviation, msg->joint_position_standard_deviation);
    _replace_vector<7>(&joint_velocity_standard_deviation, msg->joint_velocity_standard_deviation);
    _replace_vector<2>(&pole_angle_standard_deviation, msg->pole_angle_standard_deviation);

    // Reset
    _replace_double(&hardware_reset_duration, msg->hardware_reset_duration);
    _replace_vector<7>(&hardware_reset_stiffness, msg->hardware_reset_stiffness);
    _replace_vector<7>(&hardware_reset_damping, msg->hardware_reset_damping);

    // Control
    _replace_vector<8>(&control, msg->control);
}

void franka_pole::Parameters::_replace_uint(unsigned int *dest, unsigned int source)
{
    if (source != 0) *dest = source;
}

void franka_pole::Parameters::_replace_double(double *dest, double source)
{
    if (!isnan(source)) *dest = source;
}

void franka_pole::Parameters::_replace_quaternion(Eigen::Quaterniond *dest, const boost::array<double, 4> &source)
{
    if (!isnan(source[0])) dest->w() = source[0];
    if (!isnan(source[1])) dest->x() = source[1];
    if (!isnan(source[2])) dest->y() = source[2];
    if (!isnan(source[3])) dest->z() = source[3];
}

template<int N> void franka_pole::Parameters::_replace_vector(Eigen::Matrix<double, N, 1> *dest, const boost::array<double, N> &source)
{
    for (size_t i = 0; i < N; i++)
    {
        if (!isnan(source[i])) (*dest)(i) = source[i];
    }
}

franka_pole::Parameters::Parameters(const ParameterReader &reader, ros::NodeHandle &node_handle) :
    arm_id(reader.arm_id()),
    simulated(reader.simulated()),
    model(reader.model()),

    franka_period(reader.franka_period()),
    pole_period(reader.pole_period()),
    command_period(reader.command_period()),
    publish_period(reader.publish_period()),
    controller_period(reader.controller_period()),

    target_effector_position(reader.target_effector_position()),
    target_effector_orientation(reader.target_effector_orientation()),
    target_joint0_position(reader.target_joint0_position()),
    min_effector_position(reader.min_effector_position()),
    max_effector_position(reader.max_effector_position()),

    initial_effector_position(reader.initial_effector_position()),
    initial_effector_orientation(reader.initial_effector_orientation()),
    initial_joint0_position(reader.initial_joint0_position()),
    initial_pole_positions(reader.initial_pole_positions()),
    initial_pole_velocities(reader.initial_pole_velocities()),

    outbound_translation_stiffness(reader.outbound_translation_stiffness()),
    outbound_translation_damping(reader.outbound_translation_damping()),
    outbound_rotation_stiffness(reader.outbound_rotation_stiffness()),
    outbound_rotation_damping(reader.outbound_rotation_damping()),
    translation_stiffness(reader.translation_stiffness()),
    translation_damping(reader.translation_damping()),
    rotation_stiffness(reader.rotation_stiffness()),
    rotation_damping(reader.rotation_damping()),

    joint_stiffness(reader.joint_stiffness()),
    joint_damping(reader.joint_damping()),

    nullspace_stiffness(reader.nullspace_stiffness()),
    nullspace_damping(reader.nullspace_damping()),

    dynamics(reader.dynamics()),

    pole_angle_filter(reader.pole_angle_filter()),
    pole_dangle_filter(reader.pole_dangle_filter()),

    joint_position_standard_deviation(reader.joint_position_standard_deviation()),
    joint_velocity_standard_deviation(reader.joint_velocity_standard_deviation()),
    pole_angle_standard_deviation(reader.pole_angle_standard_deviation()),

    hardware_reset_duration(reader.hardware_reset_duration()),
    hardware_reset_stiffness(reader.hardware_reset_stiffness()),
    hardware_reset_damping(reader.hardware_reset_damping()),

    control(reader.control())
{
    _subscriber = node_handle.subscribe("/franka_pole/command_parameters", 10, &Parameters::_callback, this, ros::TransportHints().reliable().tcpNoDelay());
}