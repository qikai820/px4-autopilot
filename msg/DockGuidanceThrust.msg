# Optional final collective throttle control, consumed only by the matching active mode.
uint64 timestamp                 # time since system start (microseconds)
uint64 activation_timestamp      # unique timestamp of this Dock mode entry
float32 thrust                   # normalized final body-Z collective throttle; 0 disables
float32 slew_rate                # maximum normalized throttle change per second
uint8 source_id                  # registered Dock navigation mode ID
bool valid                      # active, airborne and healthy guidance
