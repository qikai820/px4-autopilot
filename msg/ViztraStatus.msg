# Health and parser state for the Viztra serial observation driver.

uint64 timestamp                 # time since system start (microseconds)
uint64 last_frame_timestamp      # time of the last syntactically valid frame (microseconds)
uint64 last_valid_timestamp      # time of the last valid observation (microseconds)

uint32 received_frames           # accepted 4.1.3 frames
uint32 valid_observations        # accepted frames with a valid running observation
uint32 checksum_errors           # frames rejected because of checksum failure
uint32 format_errors             # frames rejected because of header, command, length, or value errors

uint8 last_protocol_status       # raw status byte from the last accepted frame
bool connected                   # a valid serial frame arrived within the configured timeout
bool observation_valid           # a valid observation arrived within the configured timeout
