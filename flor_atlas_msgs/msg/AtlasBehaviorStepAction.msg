#
# This message was adapted from BDI Atlas Behavior API (2.8.0)
#

# ToeOffSetting
uint8 TOE_OFF_DISABLE      = 0  # disable toe-off
uint8 TOE_OFF_ENABLE       = 1  # enable toe-off as needed
uint8 TOE_OFF_FORCE_ENABLE = 2  # force a toe-off

float32 step_duration           # duration of the single support phase (STEP_SUBSTATE_STEPPING)

float32 sway_duration           # duration of the double support phase (STEP_SUBSTATE_SWAYING)
                                # Ignored if it is less than 0.2

float32 swing_height            # step apex above the lift height while swinging the leg
                                # This is the additional height above the lift height that the foot 
                                # will reach while swinging to its final x,y position
                                # Ignored if it is less than 0.1

float32 lift_height             # height to lift vertically before moving the foot in x,y
                                # This is computed to be above the maximum of the start 
                                # and end locations of the foot.
                                # Ignored if it is less than 0.1

int32 toe_off                   # specify whether toe-off is allowed or not during swaying
                                # Should be set to TOE_OFF_DISABLE, TOE_OFF_ENABLE, or TOE_OFF_FORCE_ENABLE
                                # For feedback, this will be set to TOE_OFF_DISABLE or TOE_OFF_ENABLE, based
                                # on whether the step is using a toe-off or not.

float32 knee_nominal            # Nominal knee angle during the step.
                                # Used to change the desired amount of knee flex while walking.
                                # Useful to tweak when running into ankle limits on sloped terrain.
                                # Ignored if less than 0.3

float32 max_body_accel          # Maximum body acceleration to determine minimum sway duration
                                # The minimum sway duration will be determined by this maximum allowable
                                # body acceleration.  Increase this value to allow the robot to move
                                # faster, but doing so may cause instability.
                                # Ignored if zero

float32 max_foot_vel            # Maximum foot velocity to determine minimum step duration
                                # The minimum step duration will be determined by this maximum
                                # foot velocity.  Increase this value to allow the robot to move faster,
                                # but doing so may destabilize the robot.
                                # Ignored if zero

float32 sway_end_dist           # Distance short of the foot to aim for at the end of sway (in meters)
                                # Tuning parameter for swaying.  Increase this value if the robot 
                                # falls to the outside at the end of a sway. 
                                # Ignored if not less than +- 5cm

float32 step_end_dist           # Distance to lean into the step before the foot comes down (in meters)
                                # Tuning parameter for stepping.  Increase this value if the foot 
                                # touchdown tends push the robot over unstably.
                                # Ignored if not less than +- 5cm
