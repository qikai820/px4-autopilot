# Position-controller force reference. Normalized by total installed maximum thrust.
uint64 timestamp
uint64 timestamp_sample
float32[3] xyz # World NED; transported to body FRD by vector_rate_control
