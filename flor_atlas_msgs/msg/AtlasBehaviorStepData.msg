#
# This message was adapted from BDI Atlas Behavior API (2.8.0)
#

std_msgs/Header header

int32 step_index                # Step index.
                                # Start this at 1 at the beginning of Walk and Step behaviors, and
                                # monotonically increase it for each step during walking.  Reset to 1
                                # if the robot re-enters Walk or Step.
                                # To specify "not a step", i.e., that an instance doesn't contain
                                # data that should be considered a step, set the step_index to -1.

int32 foot_index                # Foot index (Left/Right); see AtlasFootId enum.
                                # For the Walk behavior, the foot index needs to alternate with each
                                # step; consecutive steps with the same foot are not allowed.

float32 duration                # Step duration

geometry_msgs/Vector3 position  # Foot position, in Atlas world frame.

float32 yaw                     # Foot orientation yaw component, in Atlas world frame.

geometry_msgs/Vector3 normal    # Foot ground normal, in Atlas world frame.

float32 swing_height            # Step apex swing height as measured from the midpoint between the feet.
