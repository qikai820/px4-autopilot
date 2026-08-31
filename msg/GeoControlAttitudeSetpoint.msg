uint64 timestamp
uint64 timestamp_sample
float32[4] q
float32 roll
float32 pitch
float32 yaw
float32[3] rates
# Accumulated torque corrections from the attitude-error and angular-rate-error
# integrators, expressed in the vehicle body frame [N m].
float32[3] attitude_integral
float32[3] rate_integral
# Estimated external body torque and the compensation applied to the controller [N m].
float32[3] disturbance_torque
float32[3] adaptive_torque
