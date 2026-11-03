# Feedforward paired by timestamp with vehicle_rates_setpoint.
uint64 timestamp
uint64 timestamp_sample
uint64 reference_timestamp
float32[3] omega_ned # rad/s, reference angular velocity in local NED
float32[3] alpha_ned # rad/s^2, derivative in local NED (transport in gyro loop)
uint8 quat_reset_counter
bool valid
