#include <franka_pole/parameters.h>
#include <franka_pole/parameter_reader.h>

bool PUBLISH = false;
int IGNORE = 0;

void franka_pole::Parameters::_receive(const CommandParameters::ConstPtr &msg)
{
    std::lock_guard<std::mutex>(*_mutex);

    // Periods
    _receive_uint(&franka_period, msg->franka_period);
    _receive_uint(&pole_period, msg->pole_period);
    _receive_uint(&command_period, msg->command_period);
    _receive_uint(&publish_period, msg->publish_period);
    _receive_uint(&controller_period, msg->controller_period);

    // Target state and constraints
    _receive_vector<3>(&target_effector_position, msg->target_effector_position);
    _receive_quaternion(&target_effector_orientation, msg->target_effector_orientation);
    _receive_double(&target_joint0_position, msg->target_joint0_position);
    _receive_vector<3>(&min_effector_position, msg->min_effector_position);
    _receive_vector<3>(&max_effector_position, msg->max_effector_position);

    // Initial state
    _receive_vector<3>(&initial_effector_position, msg->initial_effector_position);
    _receive_quaternion(&initial_effector_orientation, msg->initial_effector_orientation);
    _receive_double(&initial_joint0_position, msg->initial_joint0_position);
    _receive_vector<2>(&initial_pole_positions, msg->initial_pole_positions);
    _receive_vector<2>(&initial_pole_velocities, msg->initial_pole_velocities);

    // Stiffness
    _receive_vector<3>(&outbound_translation_stiffness, msg->outbound_translation_stiffness);
    _receive_vector<3>(&outbound_translation_damping, msg->outbound_translation_damping);
    _receive_vector<3>(&outbound_rotation_stiffness, msg->outbound_rotation_stiffness);
    _receive_vector<3>(&outbound_rotation_damping, msg->outbound_rotation_damping);
    _receive_vector<3>(&translation_stiffness, msg->translation_stiffness);
    _receive_vector<3>(&translation_damping, msg->translation_damping);
    _receive_vector<3>(&rotation_stiffness, msg->rotation_stiffness);
    _receive_vector<3>(&rotation_damping, msg->rotation_damping);

    _receive_vector<7>(&joint_stiffness, msg->joint_stiffness);
    _receive_vector<7>(&joint_damping, msg->joint_damping);

    _receive_vector<7>(&nullspace_stiffness, msg->nullspace_stiffness);
    _receive_vector<7>(&nullspace_damping, msg->nullspace_damping);

    _receive_double(&dynamics, msg->dynamics);
    pure_dynamics = msg->pure_dynamics;

    // Filters
    _receive_double(&pole_angle_filter, msg->pole_angle_filter);
    _receive_double(&pole_dangle_filter, msg->pole_dangle_filter);

    // Noise
    _receive_vector<7>(&joint_position_standard_deviation, msg->joint_position_standard_deviation);
    _receive_vector<7>(&joint_velocity_standard_deviation, msg->joint_velocity_standard_deviation);
    _receive_vector<2>(&pole_angle_standard_deviation, msg->pole_angle_standard_deviation);

    // Reset
    _receive_double(&hardware_reset_duration, msg->hardware_reset_duration);
    _receive_vector<7>(&hardware_reset_stiffness, msg->hardware_reset_stiffness);
    _receive_vector<7>(&hardware_reset_damping, msg->hardware_reset_damping);

    // Control
    _receive_vector<8>(&control, msg->control);

    if (_publish && _changed) { _publisher.publish(msg); _changed = false; }
}

void franka_pole::Parameters::_send()
{
    CommandParameters command;

    // Periods
    _send_uint(franka_period, &command.franka_period);
    _send_uint(pole_period, &command.pole_period);
    _send_uint(command_period, &command.command_period);
    _send_uint(publish_period, &command.publish_period);
    _send_uint(controller_period, &command.controller_period);

    // Target state and constraints
    _send_vector<3>(target_effector_position, &command.target_effector_position);
    _send_quaternion(target_effector_orientation, &command.target_effector_orientation);
    _send_double(target_joint0_position, &command.target_joint0_position);
    _send_vector<3>(min_effector_position, &command.min_effector_position);
    _send_vector<3>(max_effector_position, &command.max_effector_position);

    // Initial state
    _send_vector<3>(initial_effector_position, &command.initial_effector_position);
    _send_quaternion(initial_effector_orientation, &command.initial_effector_orientation);
    _send_double(initial_joint0_position, &command.initial_joint0_position);
    _send_vector<2>(initial_pole_positions, &command.initial_pole_positions);
    _send_vector<2>(initial_pole_velocities, &command.initial_pole_velocities);

    // Stiffness
    _send_vector<3>(outbound_translation_stiffness, &command.outbound_translation_stiffness);
    _send_vector<3>(outbound_translation_damping, &command.outbound_translation_damping);
    _send_vector<3>(outbound_rotation_stiffness, &command.outbound_rotation_stiffness);
    _send_vector<3>(outbound_rotation_damping, &command.outbound_rotation_damping);
    _send_vector<3>(translation_stiffness, &command.translation_stiffness);
    _send_vector<3>(translation_damping, &command.translation_damping);
    _send_vector<3>(rotation_stiffness, &command.rotation_stiffness);
    _send_vector<3>(rotation_damping, &command.rotation_damping);

    _send_vector<7>(joint_stiffness, &command.joint_stiffness);
    _send_vector<7>(joint_damping, &command.joint_damping);

    _send_vector<7>(nullspace_stiffness, &command.nullspace_stiffness);
    _send_vector<7>(nullspace_damping, &command.nullspace_damping);

    _send_double(dynamics, &command.dynamics);
    command.pure_dynamics = pure_dynamics;

    // Filters
    _send_double(pole_angle_filter, &command.pole_angle_filter);
    _send_double(pole_dangle_filter, &command.pole_dangle_filter);

    // Noise
    _send_vector<7>(joint_position_standard_deviation, &command.joint_position_standard_deviation);
    _send_vector<7>(joint_velocity_standard_deviation, &command.joint_velocity_standard_deviation);
    _send_vector<2>(pole_angle_standard_deviation, &command.pole_angle_standard_deviation);

    // Reset
    _send_double(hardware_reset_duration, &command.hardware_reset_duration);
    _send_vector<7>(hardware_reset_stiffness, &command.hardware_reset_stiffness);
    _send_vector<7>(hardware_reset_damping, &command.hardware_reset_damping);

    // Control
    _send_vector<8>(control, &command.control);

    _publisher.publish(command);
}

void franka_pole::Parameters::_receive_uint(unsigned int *dest, unsigned int source)
{
    if (source != 0) { if (*dest != source) _changed = true; *dest = source; }
}

void franka_pole::Parameters::_receive_double(double *dest, double source)
{
    if (!isnan(source)) { if (*dest != source) _changed = true; *dest = source; }
}

void franka_pole::Parameters::_receive_quaternion(Eigen::Quaterniond *dest, const boost::array<double, 4> &source)
{
    if (!isnan(source[0])) { if (dest->w() != source[0]) _changed = true; dest->w() = source[0]; }
    if (!isnan(source[1])) { if (dest->x() != source[1]) _changed = true; dest->x() = source[1]; }
    if (!isnan(source[2])) { if (dest->y() != source[2]) _changed = true; dest->y() = source[2]; }
    if (!isnan(source[3])) { if (dest->z() != source[3]) _changed = true; dest->z() = source[3]; }
}

template<int N> void franka_pole::Parameters::_receive_vector(Eigen::Matrix<double, N, 1> *dest, const boost::array<double, N> &source)
{
    for (size_t i = 0; i < N; i++)
    {
        if (!isnan(source[i])) { if ((*dest)(i) != source[i]) _changed = true; (*dest)(i) = source[i]; }
    }
}

void franka_pole::Parameters::_send_uint(unsigned int source, unsigned int *dest)
{
    *dest = source;
}

void franka_pole::Parameters::_send_double(double source, double *dest)
{
    *dest = source;
}

void franka_pole::Parameters::_send_quaternion(const Eigen::Quaterniond &source, boost::array<double, 4> *dest)
{
    (*dest)[0] = source.w();
    (*dest)[1] = source.x();
    (*dest)[2] = source.y();
    (*dest)[3] = source.z();
}

template<int N> void franka_pole::Parameters::_send_vector(const Eigen::Matrix<double, N, 1> &source, boost::array<double, N> *dest)
{
    for (size_t i = 0; i < N; i++)
    {
        (*dest)[i] = source(i);
    }
}

franka_pole::Parameters::Parameters(std::mutex *mutex, const ParameterReader &reader, ros::NodeHandle &node_handle, bool publish) :
    _mutex(mutex),
    _publish(publish),
    _changed(false),

    namespacee(reader.namespacee()),
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
    pure_dynamics(reader.pure_dynamics()),

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
    // Publishing parameters once
    if (publish)
    {
        _publisher = node_handle.advertise<franka_pole::CommandParameters>("/" + namespacee + "/command_parameters", 10, true);
        _send();
    }

    // Subscribing
    _subscriber = node_handle.subscribe("/" + namespacee + "/command_parameters", 10, &Parameters::_receive, this, ros::TransportHints().reliable().tcpNoDelay());
}