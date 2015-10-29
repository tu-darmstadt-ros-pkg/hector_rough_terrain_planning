#include <flor_footstep_plan_vis/flor_footstep_plan_vis.h>

namespace flor_footstep_plan_vis
{
FootstepPlannerVis::FootstepPlannerVis(ros::NodeHandle &nh)
{
  nh.param("foot/size/x", foot_size.x, 0.26);
  nh.param("foot/size/y", foot_size.y, 0.13);
  nh.param("foot/size/z", foot_size.z, 0.015);
  nh.param("foot/origin_shift/x", foot_origin_shift.x, 0.0);
  nh.param("foot/origin_shift/y", foot_origin_shift.y, 0.0);
  nh.param("foot/origin_shift/z", foot_origin_shift.z, 0.0);
  nh.param("foot/separation", foot_seperation, 0.23);

  nh.param("upper_body/size/x", upper_body_size.x, 0.7);
  nh.param("upper_body/size/y", upper_body_size.y, 1.1);
  nh.param("upper_body/size/z", upper_body_size.z, 0.0);
  nh.param("upper_body/origin_shift/x", upper_body_origin_shift.x, 0.0);
  nh.param("upper_body/origin_shift/y", upper_body_origin_shift.y, 0.0);
  nh.param("upper_body/origin_shift/z", upper_body_origin_shift.z, 0.0);
}

FootstepPlannerVis::~FootstepPlannerVis()
{}

void FootstepPlannerVis::publishFeetStartPosesVis(ros::Publisher &pub, const flor_footstep_planner_msgs::FeetPoses &feet_poses)
{
  if (pub.getNumSubscribers() > 0)
  {
    // clear old MarkerArrays
    for (visualization_msgs::MarkerArray::_markers_type::iterator itr = last_foot_pose_start_marker_array.markers.begin(); itr != last_foot_pose_start_marker_array.markers.end(); itr++)
    {
      visualization_msgs::Marker &marker(*itr);
      marker.header.stamp = ros::Time::now();
      marker.action = visualization_msgs::Marker::DELETE;
    }
    pub.publish(last_foot_pose_start_marker_array);

    // visualize start foot positions
    last_foot_pose_start_marker_array.markers.clear();

    // initialize marker
    visualization_msgs::Marker marker;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.5;
    marker.color.a = 0.6;

    flor_footstep_planner_msgs::StepTarget step_target;
    step_target.header.stamp = ros::Time::now();
    //step_target.header.frame_id = feet_poses.left.header.frame_id;
    //step_target.foot.header = step_target.header;

    // convert left foot pose
    //step_target.foot.header.frame_id = feet_poses.left.header.frame_id;
    //step_target.foot_index = flor_atlas_msgs::AtlasBehaviorFootData::FOOT_LEFT;
    //step_target.foot.position = feet_poses.left.position;
    //step_target.foot.yaw = feet_poses.left.yaw;
    //step_target.foot.normal = feet_poses.left.normal;
    //stepToMarker(step_target, marker);
    marker.id = 0;
    last_foot_pose_start_marker_array.markers.push_back(marker);

    // convert right foot pose
    //step_target.foot.header.frame_id = feet_poses.right.header.frame_id;
    //step_target.foot_index = flor_atlas_msgs::AtlasBehaviorFootData::FOOT_RIGHT;
    //step_target.foot.position = feet_poses.right.position;
    //step_target.foot.yaw = feet_poses.right.yaw;
    //step_target.foot.normal = feet_poses.right.normal;
    //stepToMarker(step_target, marker);
    marker.id = 1;
    last_foot_pose_start_marker_array.markers.push_back(marker);

    pub.publish(last_foot_pose_start_marker_array);
  }
}
}
