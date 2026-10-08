# Relative guidance diagnostics, separate from the absolute target observer.
uint64 timestamp
float32[3] relative_position
float32[3] relative_rate
float32 trajectory_miss
float32 trajectory_tgo
float32 frozen_miss_bound
float32 frozen_time_upper
float32 observation_age
float32 plan_age
uint32 updates
uint32 planner_runtime_us
float32[3] response_bias
float32[3] drag_coefficient
float32[3] linear_drag # effective NED linear resistance, 1/s
float32 response_uncertainty
float32 thrust_acceleration
bool response_valid
bool active
bool rate_ready
bool trajectory_valid
bool terminal_committed
