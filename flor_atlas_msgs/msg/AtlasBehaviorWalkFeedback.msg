#
# This message was adapted from BDI Atlas Behavior API (2.8.0)
#

# AtlasBehaviorWalkFlags
uint8 WALK_OKAY                            = 0
uint8 WALK_SUBSTATE_SWAYING                = 1
uint8 WALK_SUBSTATE_STEPPING               = 2
uint8 WALK_SUBSTATE_CATCHING               = 4
uint8 WALK_WARNING_INSUFFICIENT_STEP_DATA  = 8
uint8 WALK_ERROR_INCONSISTENT_STEPS        = 16

float32 t_step_rem                                  # Estimated step time remaining.

int32 current_step_index                            # Current step index.
                                                    # Step 0 is the position of the foot at the beginning of Walk. Step
                                                    # 1 is the first user-specified foot position, etc.
                                                    # The step index will increase as Walk progresses. It will reset to
                                                    # 0 on behavior changes.

int32 next_step_index_needed                        # Next step index needed.
                                                    # At the next touchdown, Walk will be looking for step data with
                                                    # indices starting with next_step_index_needed. This will usually
                                                    # be roughly current_step_index plus four.
                                                    # This variable should be watched to know when the next step should
                                                    # be queued.

uint32 status_flags                                 # Bit-wise array of AtlasBehaviorWalkFlags values indicating
                                                    # All flags are 0 when the behavior begins, and will be set to 1
                                                    # at appropriate times.
                                                    # All flags will latch unless otherwise noted. "Latch" means that
                                                    # once the flag is set to 1, it won't be re-set to 0 until the
                                                    # behavior is left and re-entered.
                                                    # To check a flag, use something like:
                                                    # <pre>
                                                    #    if (walk_feedback.status_flags & WALK_ERROR_INCONSISTENT_STEPS)
                                                    #        do_something_about_the_error();
                                                    # </pre>
                                                    # \e Flags:
                                                    # - WALK_SUBSTATE_SWAYING
                                                    #     Walk is preparing to take a its first step by swaying the body.
                                                    #     This flag does not latch. Only one of WALK_SUBSTATE_SWAYING,
                                                    #     WALK_SUBSTATE_STEPPING, or WALK_SUBSTATE_CATCHING will be set
                                                    #     at any given time.
                                                    # - WALK_SUBSTATE_STEPPING
                                                    #     Walk is actively stepping. This flag does not latch.
                                                    # - WALK_SUBSTATE_CATCHING
                                                    #     Walk has stopped stepping and is preparing to go back to Stand.
                                                    #     This flag does not latch.
                                                    # - WALK_WARNING_INSUFFICIENT_STEP_DATA
                                                    #     Walk was unable to find the number of steps it was looking for
                                                    #     and has triggered an attempt to go back to Stand. (Does not
                                                    #     latch.)
                                                    # - WALK_ERROR_INCONSISTENT_STEPS
                                                    #     Walk detected the user tried to take 2 steps with the same
                                                    #     foot, and attempt to go back into Stand.

flor_atlas_msgs/AtlasBehaviorStepData[4] step_queue_saturated       # Saturated copy of step_queue from user.
                                                                    # Walk will do its best to reach the desired step data, but may have
                                                                    # to modify data to fit within constraints. The modified data is
                                                                    # contained in this variable.

flor_atlas_msgs/AtlasBehaviorWalkSpec[4] walk_spec_queue_saturated  # Saturated copy of walk_spec_queue from user.
                                                                    # Walk will do its best to reach the desired step data, but may have
                                                                    # to modify data to fit within constraints. The modified data is
                                                                    # contained in this variable.
