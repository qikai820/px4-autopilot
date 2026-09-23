uint64 timestamp
float32 bbox_edge_ratio               # largest normalized target-box edge occupancy
float32 cross_sigma_m                 # predicted transverse position uncertainty at plane [m]
float32 cross_velocity_sigma_m_s      # transverse target-velocity uncertainty [m/s]
float32 tgo_s                         # predicted time to approach plane [s]
bool prediction_ready                 # short-horizon estimate passed the gate and planner
bool terminal_ready                   # bounded near-field pass committed
uint8 reject_reason                   # PredictionReject values in PredictiveCorrection.hpp
