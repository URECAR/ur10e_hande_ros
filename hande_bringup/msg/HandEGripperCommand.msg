# Command message for the Robotiq Hand-E gripper

# Command values
bool rACT      # Activation status: true = activate, false = deactivate
bool rGTO      # Go to position: true = go to position, false = stop
bool rATR      # Automatic release: true = auto release, false = normal
uint8 rPR      # Position request: 0-255 (0 = fully open, 255 = fully closed)
uint8 rSP      # Speed: 0-255
uint8 rFR      # Force: 0-255