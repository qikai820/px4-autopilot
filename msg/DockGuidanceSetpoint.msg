# Guidance state produced by the dock-guidance module.

uint64 timestamp                     # time since system start (microseconds)

float32 closing_speed_m_s            # ground speed projected onto the earth-frame LOS [m/s]
float32 target_speed_m_s             # commanded approach-speed magnitude [m/s]
float32[3] velocity_setpoint_ned_m_s # NED velocity setpoint sent to mc_pos_control [m/s]
float32[3] relative_position_ned_m   # target position relative to vehicle in NED [m]
float32 relative_distance_m          # target slant range [m]
float32 fov_ratio                    # largest normalized image offset; 1 is the image edge

float32 camera_yaw_deg               # camera-frame yaw error [deg] (right-positive)
float32 camera_pitch_deg             # camera-frame pitch error [deg] (up-positive)

uint8 stage                          # 0=NONE 1=ALIGN 2=ACCEL 3=MAX_SPEED 4=BRAKE 5=RECOVERY
bool velocity_valid                  # fused local velocity is fresh and finite
bool relative_position_valid         # fresh finite target-relative NED position is available
bool vision_degraded                 # bridging a short visual dropout
bool valid                           # true while the module has a usable target
