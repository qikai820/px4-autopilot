# Guidance state produced by the dock-guidance module.

uint64 timestamp                     # time since system start (microseconds)

float32 closing_speed_m_s            # ground speed projected onto the earth-frame LOS [m/s]
float32 target_speed_m_s             # commanded approach-speed magnitude [m/s]
float32[3] velocity_setpoint_ned_m_s # NED velocity setpoint sent to mc_pos_control [m/s]
float32[3] tracking_velocity_ned_m_s # image-servo correction perpendicular to target LOS [m/s]
float32[3] relative_position_ned_m   # target position relative to vehicle in NED [m]
float32 relative_distance_m          # target slant range [m]
float32 fov_ratio                    # largest normalized image offset; 1 is the image edge
float32 fov_yaw_ratio                # normalized horizontal image offset
float32 fov_pitch_ratio              # normalized vertical image offset

float32 camera_yaw_deg               # camera-frame yaw error [deg] (right-positive)
float32 camera_pitch_deg             # camera-frame pitch error [deg] (up-positive)
float32 camera_mount_pitch_deg       # commanded body-relative camera pitch [deg]
float32 yaw_setpoint_deg             # slew-limited vehicle yaw setpoint [deg]

uint8 stage                          # 0=NONE 1=ALIGN 2=ACCEL 3=MAX_SPEED 4=BRAKE 5=RECOVERY 6=EMERGENCY_RECOVERY
bool velocity_valid                  # fused local velocity is fresh and finite
bool relative_position_valid         # fresh finite target-relative NED position is available
bool camera_servo_enabled            # true when slow camera pitch-servo tracking is enabled
bool vision_degraded                 # bridging a short visual dropout
bool fault_latched                   # requires pilot mode deselection before Dock can run again
bool valid                           # true while the module has a usable target
