uint8 status       # Raw status byte
uint8 fault        # Fault code (0 for no fault)
uint8 position     # Current position (0-255)
bool is_activated  # True if gripper is activated
bool is_moving     # True if gripper is in motion
bool object_detected    # True if an object is detected