#
# This message was adapted from BDI Atlas Behavior API (2.8.0)
#

# AtlasHandId
uint8 HAND_LEFT  = 0
uint8 HAND_RIGHT = 1

geometry_msgs/PoseStamped com       # Center of mass in hand frame (position in meters) - ignore orientation, includes header with frame id = hand
uint8                     hand      # Specify left or right in message
float32                   mass      # Mass (kg)

# -------- Future------------------------
# geometry_msgs/Vector3     Ix        # x-column of inertia matrix in hand frame
# geometry_msgs/Vector3     Iy        # y-column of inertia matrix in hand frame
# geometry_msgs/Vector3     Iz        # z-column of inertia matrix in hand frame
