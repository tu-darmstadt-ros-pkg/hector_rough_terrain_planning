std_msgs/Header header
uint8 planning_mode                                         # (see below)

uint8 MODE_WALK                   = 0                       # simple planning for flat ground (default)
uint8 MODE_STEP                   = 1                       # finer planning of each step
uint8 MODE_FEET_REALIGN_ON_CENTER = 2                       # realign feet relative to robot pose
uint8 MODE_FEET_REALIGN_ON_LEFT   = 3                       # realign feet relative to left foot
uint8 MODE_FEET_REALIGN_ON_RIGHT  = 4                       # realign feet relative to right foot
uint8 MODE_WIDE_STANCE            = 5                       # special mode for wide stance

flor_footstep_planner_msgs/StepTarget[] step_plan           # resulting general footstep plan
