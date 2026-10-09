uint64 timestamp # FC receive time, microseconds
uint64 source_time_boot_ms # QGC boot time, not in FC time domain
uint32 sequence
uint8 source_system
uint8 source_component
uint8 target_id
uint8 valid_flags # bit 0 position, bit 1 NED velocity
float64 lat # WGS84 degrees
float64 lon # WGS84 degrees
float32 alt_msl # meters
float32 vn # north velocity, m/s
float32 ve # east velocity, m/s
float32 vd # down velocity, m/s
