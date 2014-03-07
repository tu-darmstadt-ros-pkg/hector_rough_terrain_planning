#
# This message was adapted from BDI Atlas Behavior API (2.8.0)
#

int32 step_index                                # Step index
                                                # Start this at 1 at the beginning of the Step behavior, and
                                                # monotonically increase it for each step during walking.  Reset to 1
                                                # if the robot re-enters Step.
                                                # To specify "not a step", i.e., that an instance doesn't contain
                                                # data that should be considered a step, set the step_index to -1.

int32 foot_index                                # Foot index (Left/Right); see AtlasFootId enum.

flor_atlas_msgs/AtlasBehaviorFootData foot      # Foothold to step to

flor_atlas_msgs/AtlasBehaviorStepAction action  # Parameters of the stepping action
