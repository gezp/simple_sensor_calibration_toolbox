# timestamp
builtin_interfaces/Time timestamp

# sensor info
string frame_id
string sensor_topic

# child sensor info for extrinsic calibration
string child_frame_id
string child_sensor_topic

# basic info
# calibration type: ssct_camera_intrinsic_calib, etc.
string calibration_type
string command_topic

# state type
uint8 UNKNOWN = 0
uint8 READY = 1
uint8 CALIBRATING = 2
uint8 PAUSED = 3
uint8 SUCCESSED = 4
uint8 FAILED = 5

# calibration state
uint8 state 0

# calibration progress
uint8 progress 0

# calibration info
string info
