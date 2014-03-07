# The goal joint configuration. This has to contain the
# correct number of DOF corresponding to the selected
# planning group.
float64[] position

# Planning group to use, currently can be one of
# l_arm_group
# l_arm_with_torso_group
# r_arm_group
# r_arm_with_torso_group
# neck_group
string planning_group
