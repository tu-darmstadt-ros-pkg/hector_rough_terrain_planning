uint32 change_mask
# values for change mask
uint32 STEP_COST_ESTIMATOR   =   1
uint32 FOOTSTEP_SET          =   2
uint32 LOAD_MAP_STEP_COST    =   4
uint32 LOAD_GPR_STEP_COST    =   8
uint32 COLLISION_CHECK_TYPE  =  16
uint32 FOOT_SIZE             =  32
uint32 UPPER_BODY_SIZE       =  64
uint32 STANDARD_STEP_PARAMS  = 128
uint32 TERRAIN_MODEL         = 256

### STEP_COST_ESTIMATOR ########
# switch between EUCLIDEAN or GPR
uint8 step_cost_type
# values for step_cost_type
uint8 EUCLIDEAN_STEP_COST_ESTIMATOR = 0  # discrete footstep planning
uint8 GPR_STEP_COST_ESTIMATOR       = 1  # continous GPR-based planning
uint8 MAP_STEP_COST_ESTIMATOR       = 2  # continous map-based planning
uint8 BOUNDARY_STEP_COST_ESTIMATOR  = 3  # continous boundary-based planning
uint8 DYNAMICS_STEP_COST_ESTIMATOR  = 4  # continous boundary-based planning with dynamic restrictions

### FOOTSTEP_SET ########
# parameters for discret footstep planning mode
#flor_atlas_msgs/AtlasBehaviorStepData[] footstep_set            # set of footsteps (displacement vectors (in meter / rad))
float32[] footstep_cost                                         # cost for each footstep given in footstep set

### LOAD_GPR_STEP_COST ########
# map step cost file
std_msgs/String map_step_cost_file

### LOAD_MAP_STEP_COST ########
# destination of gpr file
std_msgs/String gpr_step_cost_file

### COLLISION_CHECK_TYPE ########
# collision check
uint8 collision_check_type

# values for collision_check_type (may composed via bitwise selection, e.g. foot and upper body => 3)
uint8 FEET_COLLISION        = 1
uint8 UPPER_BODY_COLLISION  = 2
uint8 FOOT_CONTACT_SUPPORT  = 4

### FOOT_SIZE ########
# setting foot size and/or upper body
geometry_msgs/Vector3 foot_size
geometry_msgs/Vector3 foot_origin_shift
float32 foot_seperation

### UPPER_BODY_SIZE ########
geometry_msgs/Vector3 upper_body_size
geometry_msgs/Vector3 upper_body_origin_shift

### TERRAIN_MODEL ########
bool use_terrain_model
uint32 min_sampling_steps_x
uint32 min_sampling_steps_y
uint32 max_sampling_steps_x
uint32 max_sampling_steps_y
float32  max_intrusion_z    
float32 max_ground_clearance 
float32 minimal_support    

### STANDARD_STEP_PARAMS ########
# standard params for steps
float32 step_duration
float32 sway_duration   # Ignored if it is less than 0.2
float32 swing_height    # Ignored if it is less than 0.1
float32 lift_height     # Ignored if it is less than 0.1
