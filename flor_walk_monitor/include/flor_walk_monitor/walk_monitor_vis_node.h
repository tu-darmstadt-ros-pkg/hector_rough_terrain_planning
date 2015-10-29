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

#ifndef FLOR_WALK_MONITOR_VIS_NODE_H__
#define FLOR_WALK_MONITOR_VIS_NODE_H__

#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/Point32.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

#include <flor_walk_monitor/walk_performance.h>

#include <flor_footstep_plan_vis/flor_footstep_plan_vis.h>

#include <flor_walk_monitor/helper.h>
#include <flor_walk_monitor/walk_monitor_mongodb.h>


namespace flor_walk_monitor
{
enum data_type { ZMP, CoP, CoP_posn_left, CoP_posn_right, CoP_force_left, CoP_force_right };

class WalkMonitorVisNode
{
public:
  WalkMonitorVisNode();
  virtual ~WalkMonitorVisNode();

//protected:
  void loadAndVisData(const ros::TimerEvent &timer_event);

  // visualization functions
  void publishVis(const walk_performance &data);
  void publishFootstepVis(const walk_performance &data);
  void publishPathVis(const walk_performance &data);
  void publishPointVecVis(ros::Publisher &pub, data_type select);

  // converter functions
  void point32ToPoint(const geometry_msgs::Point32 &point32, geometry_msgs::Point &point) const;
  void extractPointVec() const;
  //void extractFootstepPlan(/*const flor_gpr::footstep_planner_gpr_data_set &data_set,*/ std::vector<atlas_msgs::AtlasBehaviorStepData> &footstep_plan) const;

  // publisher
  ros::Publisher footstep_plan_vis_pub;
  ros::Publisher feet_poses_start_vis_pub;
  ros::Publisher footstep_path_vis_pub;
  ros::Publisher zmp_vis_pub;
  ros::Publisher cop_vis_pub;
  ros::Publisher cop_posn_left_vis_pub;
  ros::Publisher cop_posn_right_vis_pub;
  ros::Publisher cop_force_left_vis_pub;
  ros::Publisher cop_force_right_vis_pub;

  // timer
  ros::Timer timer;

  // mongodb handler
  flor_walk_monitor::WalkMonitorMongodb::Ptr mongodb;

  flor_footstep_plan_vis::FootstepPlannerVis::Ptr footstep_planner_vis;
};
}

#endif
