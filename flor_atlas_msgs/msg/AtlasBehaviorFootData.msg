#
# This message was adapted from BDI Atlas Behavior API (2.8.0)
#

std_msgs/Header header

# AtlasFootId
uint8 FOOT_LEFT  = 0
uint8 FOOT_RIGHT = 1

geometry_msgs/Vector3 position  # Foot position, in Atlas world frame.
float32 yaw                     # Foot orientation yaw component, in Atlas world frame.
geometry_msgs/Vector3 normal    # Foot ground normal, in Atlas world frame.
