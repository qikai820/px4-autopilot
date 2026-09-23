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
float32 target_size_ratio            # largest target-box dimension divided by image size
float32 terminal_time_s              # time since TERMINAL_DASH was committed [s]
float32 terminal_blind_time_s        # time since vision was lost in TERMINAL_DASH [s]

float32 camera_yaw_deg               # camera-frame yaw error [deg] (right-positive)
float32 camera_pitch_deg             # camera-frame pitch error [deg] (up-positive)
float32 camera_mount_pitch_deg       # commanded body-relative camera pitch [deg]
float32 yaw_setpoint_deg             # slew-limited vehicle yaw setpoint [deg]

uint8 stage                          # 0=NONE 1=ALIGN 2=ACCEL 3=MAX_SPEED 4=BRAKE 5=RECOVERY 6=EMERGENCY_RECOVERY 7=TERMINAL_DASH 8=PASS_PREDICTED 9=STARTUP_CLIMB 10=TRACK_CAPTURE
bool velocity_valid                  # fused local velocity is fresh and finite
bool relative_position_valid         # fresh finite target-relative NED position is available
bool camera_servo_enabled            # true when slow camera pitch-servo tracking is enabled
bool vision_degraded                 # bridging a short visual dropout
bool fault_latched                   # requires pilot mode deselection before Dock can run again
bool valid                           # true while the module has a usable target

# Dynamic-target diagnostics; these do not change the Viztra wire protocol.
bool dynamic_enabled
bool target_motion_valid             # metric scale and velocity estimate are usable
bool dynamic_reachable               # estimated following plus positive closure fits limits
bool pass_predicted                  # prediction only, NOT contact/docking confirmation
float32[3] estimated_target_velocity_ned # target ground velocity, not approach command [m/s]
float32[3] estimated_relative_ned     # distinct from measured relative_position_ned_m [m]
float32 target_velocity_error        # combined velocity standard deviation [m/s]
float32 target_position_error        # position error floor including scale uncertainty [m]
float32 relative_closing_speed       # (own velocity - estimated target velocity) dot LOS [m/s]
float32 commanded_relative_speed     # scheduled LOS closure [m/s]
float32 available_relative_speed     # feasible LOS closure after following allocation [m/s]
uint8 target_motion_state            # 0 unknown, 1 stationary evidence, 2 moving evidence
uint32 estimator_reset_count
uint8 estimator_reset_reason         # 0 lifecycle, 1 calibration, 2 mode configuration, 3 estimator gap/state, 4 local reset
uint16 estimator_updates
float32 estimator_age_s
float32 startup_remaining_m          # NAN when height reference is not active
bool startup_height_hold
bool velocity_limits_override        # speed/Z boundary prevented satisfying accel/jerk simultaneously
float32 predicted_fov_ratio
float32 capture_speed_limit          # active capture limit before final vehicle/FOV constraints
bool capture_evidence_valid          # current frame satisfies image/range/LOS evidence gate
bool capture_acceleration_ready      # continuous evidence confirmation completed
float32[3] following_command_ned
float32[3] capture_command_ned
float32[3] desired_velocity_ned       # before final acceleration/FOV planning
float32[3] command_acceleration_ned   # difference of actual published velocity commands

# Exposure alignment uses camera COMMAND history, not measured mount feedback.
uint64 exposure_timestamp             # timestamp_sample minus DOCK_CAM_DLY, microseconds
float32 sample_mount_pitch_deg        # command angle in effect at exposure [deg]
bool camera_time_aligned              # supporting camera/body history found, no visual dropout
bool prediction_valid                 # trusted approaching state within correction horizon
float32 prediction_tgo_s              # time to approach plane under nominal first-order response [s]
float32[3] prediction_miss_ned         # residual at predicted time including nominal response [m]
float32[3] prediction_dv_ned           # desired-velocity change after speed limits, before accel/FOV [m/s]
