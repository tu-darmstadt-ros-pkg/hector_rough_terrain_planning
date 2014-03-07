# The goal pose. This can be given in any tf frame
geometry_msgs/PoseStamped  pose

# Set use of obstacle avoidance on/off
std_msgs/Bool use_environment_obstacle_avoidance

# Planning group to use, currently can be one of
# l_arm_group
# l_arm_with_torso_group
# r_arm_group
# r_arm_with_torso_group
std_msgs/String planning_group

# More options could be added here