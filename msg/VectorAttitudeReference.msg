# Atomic vector reference (plane or spatial thrust capability).
# timestamp matches vehicle_attitude_setpoint; heading/bank/pitch factor Rz*Rx*Ry.
uint64 timestamp
uint64 timestamp_sample
float32[4] q_d
float32[3] omega_ned # rad/s, angular velocity of desired attitude in local NED
float32[3] alpha_ned # rad/s^2, time derivative of omega_ned in local NED
float32[3] force_raw # NED, normalized by installed thrust
float32[3] force_limited # NED, normalized by installed thrust
float32 heading # rad, Rz*Rx*Ry reference heading, NOT Euler yaw of the body
float32 bank # rad, force-derived for plane vehicles, independent for spatial vehicles
float32 pitch # rad, independent pitch within the banked frame
float32 pitch_authority # [0.2,1], pitch governor (both independent axes for spatial vehicles)
uint8 quat_reset_counter # vehicle_attitude frame epoch
bool valid
