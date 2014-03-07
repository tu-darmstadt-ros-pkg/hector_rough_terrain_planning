#
# This message was adapted from BDI Atlas Behavior API (2.8.0)
#

# AtlasHandId
int8 HAND_LEFT  = 0
int8 HAND_RIGHT = 1

uint8 HAND_POWER_OFF = 0
uint8 HAND_POWER_ON  = 1

std_msgs/Header     header    # Time stamp
int8                hand      # Specify left or right in message, or -1 to apply same to both hands
uint8               power     # 0=off, >1 = on
