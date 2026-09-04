# Visual observation reported by a Viztra-compatible tracking board.
# This topic is observational only and is not consumed by vehicle flight control.

uint64 timestamp                 # time since system start (microseconds)
uint64 timestamp_sample          # receive time of the complete serial frame (microseconds)

float32 offset_x                 # right-positive horizontal offset [px or deg, see offsets_are_angles]
float32 offset_y                 # up-positive vertical offset [px or deg, see offsets_are_angles]
float32 relative_n_m             # target position relative to vehicle, north-positive [m]
float32 relative_e_m             # target position relative to vehicle, east-positive [m]
float32 relative_d_m             # target position relative to vehicle, down-positive [m]
uint16 target_width_px           # target bounding-box width [px]
uint16 target_height_px          # target bounding-box height [px]

uint8 video_channel              # source video channel reported by the board
uint8 protocol_status            # raw 4.1.3 status byte
bool valid                       # board reports a valid, running observation
bool offsets_are_angles          # false: offset_x/y are pixels; true: they are degrees
bool relative_position_valid     # true when the extended frame contains relative NED position
