string command     # Command: "open", "close", "goto", "stop"
float32 position   # Target position in mm (for "goto" command)
uint8 speed        # Speed (0-255, 0 to use current setting)
uint8 force        # Force (0-255, 0 to use current setting)