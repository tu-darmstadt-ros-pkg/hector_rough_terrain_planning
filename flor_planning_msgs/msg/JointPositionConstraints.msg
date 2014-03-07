# Joint limits for IK that shouldn't be exceeded
# Only torso joints at this time
# Per convention:
# -pi to pi are valid constraints
# -100.0 means no constraint (use URDF joint limits)
# +100.0 means joint locked to current position

std_msgs/Float32 back_bkz_min
std_msgs/Float32 back_bkz_max
std_msgs/Float32 back_bky_min
std_msgs/Float32 back_bky_max
std_msgs/Float32 back_bkx_min
std_msgs/Float32 back_bkx_max
