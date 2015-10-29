std_msgs/Header                      header

# continuous logging
#flor_atlas_msgs/AtlasBehaviorWalkFeedback[] atlas_walk_feedback_log
#flor_atlas_msgs/AtlasBehaviorStepFeedback[] atlas_step_feedback_log
#atlas_msgs/AtlasSimInterfaceState[] atlas_sim_interface_log
#flor_control_msgs/FlorControlMode[] controller_mode_log
#flor_ocs_msgs/OCSRobotStability[] robot_stability_log

# discrete logging
flor_footstep_planner_msgs/StepTarget[] planned_footsteps       # footsteps planned by planner
flor_footstep_planner_msgs/FeetPoses feet_start_poses           # feet start poses
flor_footstep_planner_msgs/StepTarget[] bdi_planned_footsteps   # footsteps planned by BDI
flor_footstep_planner_msgs/StepTarget[] real_footsteps          # footsteps executed
#atlas_msgs/AtlasPositionData[] atlas_position_data              # positioning data of robot
#flor_ocs_msgs/OCSRobotStability[] robot_stability               # all stability data
uint8[] stability_worst_rating                                  # worst ever logged stability rating
bool has_fallen                                                 #

bool simulated                                                  # set to true if data was recorded in simulation
