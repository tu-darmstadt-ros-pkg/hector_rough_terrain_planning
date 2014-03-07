# Header containing frame_id of waypoint poses
Header header

# Waypoints given in frame as specified in header
geometry_msgs/Pose[] waypoints

# Avoids environment collisions if true
bool use_environment_obstacle_avoidance

# The planning group to use
string planning_group