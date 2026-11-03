uint64 timestamp
uint64 timestamp_sample
float32[3] rate_error # rad/s, body FRD
float32[3] alpha_feedforward # rad/s^2, transported to current body frame
float32[3] alpha_feedback # rad/s^2
float32[3] alpha_integral # rad/s^2
float32[3] torque_gyro # Nm, body FRD
float32[3] torque_command # Nm, before allocation
float32[3] torque_unallocated # Nm, final-command allocator residual
bool feedforward_valid
