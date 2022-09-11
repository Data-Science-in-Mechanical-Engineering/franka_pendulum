uint32 franka_period
uint32 pendulum_period
uint32 command_period
uint32 publish_period
uint32 controller_period
  
float64[3] target_effector_position
float64[4] target_effector_orientation
float64[7] target_joint_weights
float64[3] min_effector_position
float64[3] max_effector_position
float64[3] min_effector_velocity
float64[3] max_effector_velocity

float64[3] initial_effector_position
float64[4] initial_effector_orientation
float64[7] initial_joint_weights
float64[2] initial_pendulum_positions
float64[2] initial_pendulum_velocities

float64[3] outbound_translation_stiffness
float64[3] outbound_translation_damping
float64[3] outbound_rotation_stiffness
float64[3] outbound_rotation_damping
float64[3] translation_stiffness
float64[3] translation_damping
float64[3] rotation_stiffness
float64[3] rotation_damping
float64[7] joint_stiffness
float64[7] joint_damping
float64[7] nullspace_stiffness
float64[7] nullspace_damping
float64 dynamics
bool pure_dynamics

float64 pendulum_angle_filter
float64 pendulum_dangle_filter

float64[7] joint_position_mean
float64[7] joint_position_standard_deviation
float64[7] joint_velocity_standard_deviation
float64[2] pendulum_angle_mean
float64[2] pendulum_angle_standard_deviation

float64 hardware_reset_duration
float64[7] hardware_reset_stiffness
float64[7] hardware_reset_damping

float64[8] pendulum_control

float64 startup_time
bool test_rectangle
float64[3] test_amplitude
float64[3] test_frequency
float64[3] test_phase