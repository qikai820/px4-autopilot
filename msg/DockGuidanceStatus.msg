uint64 timestamp
float32 relative_distance_m # NAN when no usable visual range is available
uint16 flags # bit 0 registered, 1 active, 2 entry ready, 3 visual setpoint valid, 4 QGC target fresh, 5 encoder ready, 6 fault latched, 7 preposition enabled, 8 preposition active, 9 vision degraded, 10 encoder required
uint8 stage # DockGuidanceSetpoint stage, 0 when inactive
char[40] entry_reason # First blocking entry condition, or Ready
