# Target for a single stepping motion of a humanoid's leg
std_msgs/Header header
int32 step_index                            # Step index
uint8 foot_index                            # which leg to use (LEFT_FOOT/RIGHT_FOOT)
#flor_atlas_msgs/AtlasBehaviorFootData foot  # Foothold to step to
float32 step_duration                       # duration of the single support phase
float32 sway_duration                       # duration of the double support phase (optional)
float32 swing_height                        # step apex above the lift height while swinging the leg
float32 lift_height                         # height to lift vertically before moving the foot (optional)
int32 toe_off                               # specify whether toe-off is allowed or not during swaying
float32 knee_nominal                        # Nominal knee angle during the step. (optional)
float32 max_body_accel                      # Maximum body acceleration to determine minimum sway duration (optional)
float32 max_foot_vel                        # Maximum foot velocity to determine minimum step duration (optional)
float32 sway_end_dist                       # Distance short of the foot to aim for at the end of sway (in meters) (optional)
float32 step_end_dist                       # Distance to lean into the step before the foot comes down (in meters) (optional)
