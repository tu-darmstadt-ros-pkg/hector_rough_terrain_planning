#
# This message was adapted from BDI Atlas Behavior API (2.8.0)
#

flor_atlas_msgs/AtlasBehaviorStepData desired_step          # Desired position of next step. desired_step_spec is preferred.
                                                            # The first desired step location should be set before entering
                                                            # the Step behavior.

flor_atlas_msgs/AtlasBehaviorStepSpec desired_step_spec     # Desired position of next step.  Preferred method.
                                                            # The first desired step location should be set before entering
                                                            # the Step behavior.

int32 use_spec                                              # use the extended step specification
                                                            # Use the desired_step_spec data, instead of desired_step

int32 use_relative_step_height                              # Whether to use relative step height.
                                                            # If use_relative_step_height is true, step to step height variations
                                                            # will be relative to the current stance foot height. Otherwise the
                                                            # foot will try to step relative to the BD world position estimate
                                                            # which can be subject to drift.
