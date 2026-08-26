# Guidance setpoint produced by the dock-guidance module.
# All fields are in the body frame unless noted otherwise.

uint64 timestamp                     # time since system start (microseconds)

float32 roll_rate_rad_s              # body-x angular rate setpoint [rad/s]
float32 pitch_rate_rad_s             # body-y angular rate setpoint [rad/s]
float32 yaw_rate_rad_s               # body-z angular rate setpoint [rad/s]

float32 throttle                     # normalized throttle [0, 1]
float32 throttle_rate                # max throttle change per second [1/s]

float32 camera_yaw_deg               # camera-frame yaw error [deg] (right-positive)
float32 camera_pitch_deg             # camera-frame pitch error [deg] (up-positive)

uint8 stage                          # current guidance stage (0=NONE 1=PITCH_CAP 2=ANGLE_CAP 3=LOS_RATE)
bool valid                           # true while the module has a usable target
