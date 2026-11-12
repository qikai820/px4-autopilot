uint64 timestamp
uint64 timestamp_sample
float32[3] rate_error # rad/s, body FRD
float32[3] alpha_feedforward # rad/s^2, transported to current body frame
float32[3] alpha_feedback # rad/s^2, diagnostic PID torque / inertia; NOT acceleration feedback gains
float32[3] alpha_integral # rad/s^2, diagnostic integral torque / inertia
float32[3] pid_output # normalized torque, excludes inertia-scaled acceleration feedforward and gyro torque
float32[3] integral_output # normalized torque, same state/limits as rate_ctrl_status
float32[3] torque_accel_feedforward # Nm, inertia times transported reference acceleration
float32[3] torque_gyro # Nm, body FRD
float32[3] torque_command # Nm, before allocation
float32[3] torque_unallocated # Nm, final-command allocator residual
bool feedforward_valid
