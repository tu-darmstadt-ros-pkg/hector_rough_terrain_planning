# if set to true all settings below will be ignored
bool use_default_region_request

# The environment region specification (relative to robot)
#flor_perception_msgs/EnvironmentRegionRequest region_req

# The aggregation size to be used for this request.
# This indicates the number of LIDAR scans used to
# build the aggregated point cloud. A value of 0
# means default, to be selected by the server
# (currently 500 scans in worldmodel)
uint32 aggregation_size
