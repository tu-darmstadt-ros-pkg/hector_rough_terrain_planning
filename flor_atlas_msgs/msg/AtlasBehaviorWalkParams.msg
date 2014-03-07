#
# This message was adapted from BDI Atlas Behavior API (2.8.0)
#

flor_atlas_msgs/AtlasBehaviorStepData[4] step_queue         # Positions of upcoming steps. walk_spec_queue is preferred
                                                            # Steps in the array whose step_index variables are set to -1 are
                                                            # considered to not contain step data.
                                                            # Refer to the AtlasRobotInterface or AtlasSimInterface API Guide for
                                                            # details on when and how steps can be queued.

flor_atlas_msgs/AtlasBehaviorWalkSpec[4] walk_spec_queue    # Positions of upcoming steps. Preferred method.
                                                            # Steps in the array whose step_index variables are set to -1 are
                                                            # considered to not contain step data.
                                                            # Refer to the AtlasRobotInterface or AtlasSimInterface API Guide for
                                                            # details on when and how steps can be queued.

int32 use_spec                                              # use the extended step specification
                                                            # Use the walk_spec_queue data, instead of step_queue

int32 use_relative_step_height                              # Whether to use relative step height.
                                                            # If use_relative_step_height is true, step to step height variations
                                                            # will be relative to the current stance foot height. Otherwise the
                                                            # foot will try to step relative to the BD world position estimate
                                                            # which can be subject to drift.
