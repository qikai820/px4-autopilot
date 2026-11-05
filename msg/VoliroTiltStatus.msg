# Motion-based common-tilt observer. This is NOT a measured servo position.
uint64 timestamp
uint64 timestamp_sample
float32 tilt_command # rad, final servo output reconstructed as angle
float32 tilt_expected # rad, lag/rate model with the observation filter
float32 tilt_observed # rad, acceleration and pitch-dynamics observation; held when invalid
float32 tilt_error # rad, observed - expected; NaN when invalid
float32 error_duration # s, continuous valid time above threshold
bool enabled
bool armed
bool observation_valid
bool fallback # latched until disarm, including when monitoring is disabled
bool fixed_tilt # RC-selected MC position control; center and hold servo, reversible (fault fallback has priority)
bool fixed_tilt_pending # RC requested MC; vector allocation retained while returning body/rotors to level
