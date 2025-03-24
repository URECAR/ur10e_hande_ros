# Status message for the Robotiq Hand-E gripper

# Status values
bool gACT      # Initialization status: true = initialized, false = not initialized
bool gGTO      # Action status: true = gripper moving, false = gripper stopped
bool gSTA      # Gripper status: true = gripper in motion, false = gripper stopped or reached target position
bool gOBJ      # Object detection: 0 = moving, 1 = object detected, 2 = maximum opening reached, 3 = maximum closing reached
uint8 gFLT     # Fault status: 0 = no fault, other = fault code
uint8 gPR      # Position request echo: 0-255
uint8 gPO      # Actual position: 0-255
uint8 gCU      # Current: 0-255