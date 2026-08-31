uint64 timestamp
uint64 timestamp_sample

# Slow position-loop correction represented as a velocity bias [m/s].
float32[3] position_bias_velocity

# Estimated acceleration disturbance for the velocity loop [m/s^2].
float32[3] velocity_disturbance_acceleration

# Raw acceleration residual used by the observer [m/s^2].
float32[3] acceleration_observation

# Velocity-loop integrator state [m/s^2].
float32[3] velocity_integral

# Independently learned normalized collective required to hover.
float32 hover_thrust_estimate
float32 hover_thrust_observation
bool hover_thrust_learning

# Mode-transition vertical braking capture state.
bool vertical_capture_active
float32 vertical_capture_velocity

# false while the observer is reset or not learning.
bool valid

# 0: off, 1: estimate only, 2: compensate.
uint8 mode
