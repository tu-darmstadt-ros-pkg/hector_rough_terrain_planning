# Rotation center pose, preferably in world coords
geometry_msgs/PoseStamped rotation_center_pose

# total intended rotation angle around the pose x axis in rad (signed)
float32 rotation_angle

# Avoids environment collisions if true
bool use_environment_obstacle_avoidance

# Keep endeffector orientation if true (do not rotate around rotation_center_pose)
bool keep_endeffector_orientation

# The planning group to use
string planning_group