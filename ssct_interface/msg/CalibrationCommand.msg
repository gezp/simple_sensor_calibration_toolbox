# calibration command type
uint8 RESET = 0
uint8 START = 1
uint8 PAUSE = 2
uint8 SAVE = 3
uint8 CUSTOM_KEY_VAULE = 4

uint8 command

# used by CUSTOM_KEY_VAULE
string key
float32 value
