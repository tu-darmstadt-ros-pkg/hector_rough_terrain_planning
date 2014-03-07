//=================================================================================================
// Copyright (c) 2013, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef FLOR_WALK_MONITOR_NODE_H__
#define FLOR_WALK_MONITOR_NODE_H__

#include <ros/ros.h>
#include <tf/tf.h>

#include <flor_footstep_planner_msgs/FootstepPlannerParams.h>
#include <flor_footstep_planner_msgs/FootstepPlan.h>
#include <flor_footstep_planner_msgs/AtlasWalkFootstepPlan.h>
#include <flor_footstep_planner_msgs/AtlasStepFootstepPlan.h>
#include <flor_footstep_planner_msgs/PlanAtlasFootsteps.h>
#include <flor_footstep_planner_msgs/PlanAtlasStepping.h>

#include <flor_footstep_plan_vis/flor_footstep_plan_vis.h>

#include <flor_walk_monitor/walk_performance.h>

#include <flor_walk_monitor/helper.h>
#include <flor_walk_monitor/walk_monitor_logger.h>
#include <flor_walk_monitor/walk_monitor_mongodb.h>


namespace flor_walk_monitor
{
class WalkMonitorNode
{
public:
  WalkMonitorNode();
  virtual ~WalkMonitorNode();

  void reset();

protected:
  void setFootstepPlannerParams(const flor_footstep_planner_msgs::FootstepPlannerParamsConstPtr &params);
  void setWalkFootstepPlan(const flor_footstep_planner_msgs::AtlasWalkFootstepPlanConstPtr &footstep_plan);
  void setStepFootstepPlan(const flor_footstep_planner_msgs::AtlasStepFootstepPlanConstPtr &footstep_plan);

  template <typename T> void setFootstepPlan(const T &footstep_plan, uint8_t planning_mode)
  {
    // convert plan
    flor_footstep_planner_msgs::FootstepPlan plan;
    plan.header = footstep_plan.header;
    plan.planning_mode = planning_mode;
    //flor_footstep_planner_msgs::convertPlan(footstep_plan.step_plan, plan.step_plan);

    // give converted plan to logger
    logger->setFootstepPlan(plan);

    // publish visualization
    footstep_plan_vis->publishFootstepPlanVis(footstep_plan_vis_pub, *(logger->getFootstepPlan()));
    footstep_plan_vis->publishPathVis(footstep_path_vis_pub, *(logger->getFootstepPlan()));
    footstep_plan_vis->publishFootstepPlanBodyVis(footstep_path_body_vis_pub, *(logger->getFootstepPlan()));
  }

  void setFootstepStart(const flor_footstep_planner_msgs::FeetPosesConstPtr &feet_start_poses);

  void saveLogToDB();
  void saveLogToDBAndReset(const ros::TimerEvent &timer_event = ros::TimerEvent());

  void checkWalkStatus();

  // subscribers
  ros::Subscriber footstep_planner_params_sub;
  ros::Subscriber footstep_plan_walk_sub;
  ros::Subscriber footstep_plan_step_sub;
  ros::Subscriber footstep_start_sub;
  ros::Subscriber transform_pose_sub_; //For subscribing to transform between BDI and world

  // publisher
  ros::Publisher footstep_plan_walk_pub;
  ros::Publisher footstep_plan_step_pub;
  ros::Publisher footstep_plan_vis_pub;
  ros::Publisher feet_poses_start_vis_pub;
  ros::Publisher footstep_path_vis_pub;
  ros::Publisher footstep_path_body_vis_pub;
  ros::Publisher walk_performance_pub;

  // timer
  ros::Timer timer;

  // logger
  WalkMonitorLogger::Ptr logger;

  // mongodb handler
  WalkMonitorMongodb::Ptr mongodb;

  // other parameters
  double max_step_diff;
  double max_yaw_diff;

  // start and goal poses
  geometry_msgs::Pose start_pose;
  geometry_msgs::Pose goal_pose;

  flor_footstep_plan_vis::FootstepPlannerVis::Ptr footstep_plan_vis;

  bool walk_started, walk_finished;
};
}

#endif
