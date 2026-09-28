uint64 timestamp
float32 bbox_edge_ratio               # largest normalized target-box edge occupancy
float32 cross_sigma_m                 # predicted transverse position uncertainty at plane [m]
float32 cross_velocity_sigma_m_s      # transverse target-velocity uncertainty [m/s]
float32 tgo_s                         # predicted time to approach plane [s]
float32 early_vertical_dv_m_s          # vertical capture bias after speed limits, before acceleration/FOV planner [m/s]
float32 capture_range_lower_m           # estimated range minus twice position uncertainty [m]
float32 capture_range_required_m        # minimum lower-bound range before initial capture [m]
float32 capture_fov_limit                # active image FOV ratio limit for capture [0,1]
float32 capture_los_rate_deg_s          # earth-frame line-of-sight angular speed [deg/s]
float32 capture_los_rate_limit_deg_s    # acquisition LOS-rate limit [deg/s]
float32 capture_evidence_time_s         # continuous capture evidence duration [s]
float32 capture_confirmation_required_s # required continuous evidence duration [s]
float32 camera_image_pitch_rate_deg_s   # filtered camera-frame image pitch rate [deg/s]
float32 camera_pitch_fov_weight         # FOV-based pitch translation assist [0,1]
float32 camera_pitch_rate_weight        # rate-shortfall pitch translation assist [0,1]
float32 camera_pitch_final_weight       # applied pitch translation assist [0,1]
bool capture_gate_evaluated             # capture gates evaluated on this control update
bool capture_range_fresh                # mature estimator with recent metric range
bool capture_not_hard                   # hard FOV protection inactive
bool capture_fov_ok                     # image inside acquisition or latched soft boundary
bool capture_los_rate_ok                # earth-frame LOS rate below acquisition limit or bypassed after capture
bool capture_range_ok                   # lower-bound range above minimum or pass-through latched
bool capture_pass_through_latched       # pass-through capture confirmed on entry to gate evaluation
bool capture_limit_enabled             # initial capture speed limit is positive
bool capture_evidence_valid            # all capture evidence gates passed
bool capture_ready                     # continuous evidence exceeded DOCK_ALIGN_T
bool prediction_ready                 # short-horizon estimate passed the gate and planner
bool terminal_ready                   # bounded near-field pass committed
uint8 reject_reason                   # PredictionReject values in PredictiveCorrection.hpp
