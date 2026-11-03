# Command reconstruction and optional actuator prediction. No measured wrench.
uint64 timestamp
float32 tilt_command # rad
float32 tilt_estimate # rad
float32 reaction_requested # Nm, along common rotor axis
float32 reaction_filtered # Nm, before motor saturation
float32[5] thrust_command # N, after slew/reversal protection
float32[5] thrust_estimate # N, modeled motor lag
float32[3] force_command # N, body FRD
float32[3] force_estimate # N, body FRD
float32[3] torque_command # Nm, body FRD
float32[3] torque_estimate # Nm, body FRD
bool prediction_enabled
bool armed
