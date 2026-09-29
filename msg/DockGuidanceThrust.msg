# Optional final collective throttle control, consumed only by the matching active mode.
uint64 timestamp                 # time since system start (microseconds)
uint64 activation_timestamp      # unique timestamp of this Dock mode entry
float32 thrust                   # fixed final collective, or post-pass braking floor; 0 disables fixed throttle
float32 slew_rate                # maximum normalized throttle change per second
float32 brake_rate               # maximum thrust-direction rotation rate during braking (rad/s)
float32 brake_speed              # horizontal speed for full braking authority (m/s)
uint8 source_id                  # registered Dock navigation mode ID
bool valid                      # active, airborne and healthy guidance
bool braking                    # post-pass thrust-preserving attitude braking, with relaxed height control
