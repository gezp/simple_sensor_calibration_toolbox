# calibration command type
uint8 RESET = 0
uint8 START = 1
uint8 SAVE_RESULT = 2
uint8 COLLECT_ONCE = 3
uint8 OPTIMIZE_ONCE = 4
uint8 CUSTOM_KEY_VAULE = 5

uint8 command

# used by CUSTOM_KEY_VAULE
string key
float32 value
