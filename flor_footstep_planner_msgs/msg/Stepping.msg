# for special walk modes
uint8 planning_mode         # select planning mode (see FootstepPlan.msg)

uint8 steps # number of steps which should be executed
bool ignore_collision

float32 step_distance_forward   # [m]
float32 step_distance_sideward  # [m]
float32 turn_angle              # [rad]

bool close_step
bool swap_start_foot
bool extra_seperation

bool use_terrain
bool generate_terrain_model

bool override
float32 roll
float32 pitch
float32 swing_height
float32 lift_height
float32 knee_nominal
float32 goal_z

uint8 walk_mode

# walk modes
uint8 FORWARD       = 0 # special modes starting here
uint8 BACKWARD      = 1
uint8 STRAFE_LEFT   = 2
uint8 STRAFE_RIGHT  = 3
uint8 ROTATE_LEFT   = 4
uint8 ROTATE_RIGHT  = 5
uint8 SAMPLING      = 6
