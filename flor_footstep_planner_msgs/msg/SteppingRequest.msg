# service message for simple stepping
# start precision
uint8 start_type            # start pose precision (see FootstepPlan.srv)

geometry_msgs/Pose start
flor_footstep_planner_msgs/FeetPoses feet_start_poses

flor_footstep_planner_msgs/Stepping stepping
