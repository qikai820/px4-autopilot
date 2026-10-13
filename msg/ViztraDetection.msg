# AI detections in one Viztra 0x82 serial packet. A frame with more than 22
# targets is split across packets with the same frame_id.

uint64 timestamp                    # time the complete serial packet was received [us]
uint8 frame_id                     # wraps from 255 to 0
uint8 total_targets                # detections in the complete image frame, up to 64
uint8 packet_targets               # valid entries in the arrays below, up to 22

uint8[22] target_id                # target identifier reported by the board
uint8[22] target_type              # detector type ID
uint8[22] confidence               # detection confidence [0, 100]
uint16[22] x_px                    # top-left X coordinate [px]
uint16[22] y_px                    # top-left Y coordinate [px]
uint16[22] width_px                # detection width [px]
uint16[22] height_px               # detection height [px]
