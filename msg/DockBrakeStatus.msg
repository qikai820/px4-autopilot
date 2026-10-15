# Shared Dock braking state produced by multicopter position control.
uint64 timestamp
bool active                    # current controlled braking session
bool climb                    # energy-to-height braking selected
bool capture                  # climb stopped; arrest vertical speed and level
bool complete                 # horizontal and vertical speeds stable
bool allow_floor              # high-speed collective floor permitted in climb phase
float32 capture_floor          # lower torque-preserving floor while collecting angle [normalized]
float32 height_used           # climb from first braking point, adjusted for EKF resets [m]
