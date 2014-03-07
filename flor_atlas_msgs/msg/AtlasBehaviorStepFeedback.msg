#
# This message was adapted from BDI Atlas Behavior API (2.8.0)
#

# AtlasBehaviorStepFlags
uint8 STEP_OKAY                 = 0
uint8 STEP_SUBSTATE_SWAYING     = 1
uint8 STEP_SUBSTATE_STEPPING    = 2

float32 t_step_rem                                  # Estimated step time remaining before new step data is needed.
int32 current_step_index                            # Current step index.
                                                    # Step index 0 is the position of the foot at the beginning of Step
                                                    # behavior. Step index 1 is the first user-specified foot position,
                                                    # etc.
                                                    # The step index will increase as Step progresses. It will reset to
                                                    # 0 on behavior changes.

int32 next_step_index_needed                        # Next step index needed.
                                                    # At the next touchdown, Walk will be looking for step data with
                                                    # indices starting with next_step_index_needed. In general this will
                                                    # be current_step_index + 1.
                                                    # This variable should be watched to know when the next step should
                                                    # be queued.

uint32 status_flags                                 # Bit-wise array of AtlasBehaviorStepFlags values indicating
                                                    # Walk behavior status.
                                                    # All flags are 0 when the behavior begins, and will be set to 1
                                                    # at appropriate times.
                                                    # All flags will latch unless otherwise noted. "Latch" means that
                                                    # once the flag is set to 1, it won't be re-set to 0 until the
                                                    # behavior is left and re-entered.
                                                    # \e Flags:
                                                    # - STEP_SUBSTATE_SWAYING
                                                    #     Feet are in double support.  This flag does not latch.  Only
                                                    #     one of STEP_SUBSTATE_SWAYING or STEP_SUBSTATE_STEPPING will be
                                                    #     set at any given time.
                                                    # - STEP_SUBSTATE_STEPPING
                                                    #     Actively stepping; one foot is in the air.  This flag does not
                                                    #     latch.

flor_atlas_msgs/AtlasBehaviorStepData desired_step_saturated        # Saturated copy of desired_step from user.
                                                                    # Step will do its best to reach the desired step data, but may have
                                                                    # to modify data to fit within constraints. The modified data is
                                                                    # contained in this variable.

flor_atlas_msgs/AtlasBehaviorStepSpec desired_step_spec_saturated   # Saturated copy of desired_step_spec from user.
                                                                    # Step will do its best to reach the desired step data, but may have
                                                                    # to modify data to fit within constraints. The modified data is
                                                                    # contained in this variable.
