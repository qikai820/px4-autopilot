uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample   # time of the angle measurement (microseconds)

uint32 device_id          # PX4 device ID, including I2C bus and address
uint16 raw_angle          # 14-bit absolute angle, 0..16383
float32 angle             # absolute angle in [0, 2*pi) [rad]
