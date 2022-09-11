float64 pendulum_timestamp
float64[2] pendulum_angle
float64[2] pendulum_dangle
float64[2] pendulum_joint_angle
float64[2] pendulum_joint_dangle

float64 franka_timestamp
float64[3] franka_effector_position
float64[3] franka_effector_velocity
float64[7] franka_joint_positions
float64[7] franka_joint_velocities

float64 command_timestamp
float64[3] command_effector_position
float64[3] command_effector_velocity
float64[3] command_effector_acceleration
float64[7] command_joint_positions
float64[7] command_joint_velocities
float64[7] command_joint_accelerations

bool reset