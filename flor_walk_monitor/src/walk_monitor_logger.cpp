#include <flor_walk_monitor/walk_monitor_logger.h>

namespace flor_walk_monitor
{
WalkMonitorLogger::WalkMonitorLogger()
  : footstep_plan(flor_footstep_planner_msgs::FootstepPlanPtr())
  , feet_start_poses(flor_footstep_planner_msgs::FeetPosesPtr())
  , bdi_to_world_transformer(flor_navigation::FootstepPlanTransformer())
{
}

WalkMonitorLogger::~WalkMonitorLogger() {}

void WalkMonitorLogger::reset() {}

void WalkMonitorLogger::setFootstepPlan(const flor_footstep_planner_msgs::FootstepPlan &footstep_plan)
{
  this->footstep_plan.reset(new flor_footstep_planner_msgs::FootstepPlan(footstep_plan));
  //bdi_to_world_transformer.transformToWorld(*(this->footstep_plan));
}

void WalkMonitorLogger::setFootstepStart(const flor_footstep_planner_msgs::FeetPoses &feet_start_poses)
{
  this->feet_start_poses.reset(new flor_footstep_planner_msgs::FeetPoses(feet_start_poses));
  //bdi_to_world_transformer.transformToWorld(*(this->feet_start_poses));
}

void WalkMonitorLogger::setTransformPoseBdiToWorld(const geometry_msgs::PoseStamped::ConstPtr &transform)
{
  bdi_to_world_transformer.setTransform(*transform);
}
}
