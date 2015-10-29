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

#ifndef FLOR_WALK_MONITOR_LOGGER_H__
#define FLOR_WALK_MONITOR_LOGGER_H__

#include <map>

#include <ros/ros.h>
#include <tf/tf.h>

#include <flor_footstep_plan_transformer/footstep_plan_transformer.h>

#include <flor_footstep_planner_msgs/flor_footstep_planner_msgs.h>
#include <flor_footstep_planner_msgs/FootstepPlan.h>
#include <flor_footstep_planner_msgs/FeetPoses.h>
#include <flor_footstep_planner_msgs/StepTarget.h>

#include <flor_walk_monitor/helper.h>
#include <flor_walk_monitor/walk_performance.h>


namespace flor_walk_monitor
{
class WalkMonitorLogger
{
public:
  WalkMonitorLogger();
  virtual ~WalkMonitorLogger();

  void reset();
  bool empty() { return true; }

  // setters for saving ros messages
  void setFootstepPlan(const flor_footstep_planner_msgs::FootstepPlan &footstep_plan);
  void setFootstepStart(const flor_footstep_planner_msgs::FeetPoses &feet_start_poses);
  void setTransformPoseBdiToWorld(const geometry_msgs::PoseStamped::ConstPtr &transform);

  // getters
  const flor_footstep_planner_msgs::FootstepPlanPtr &getFootstepPlan() const { return footstep_plan; }
  const flor_footstep_planner_msgs::FeetPosesPtr &getFeetStartPoses() const { return feet_start_poses; }

  // typedefs
  typedef boost::shared_ptr<WalkMonitorLogger> Ptr;
  typedef boost::shared_ptr<const WalkMonitorLogger> ConstPtr;

protected:
  // current footstep plan
  flor_footstep_planner_msgs::FootstepPlanPtr footstep_plan;
  flor_footstep_planner_msgs::FeetPosesPtr feet_start_poses;

  // state of robot
  flor_navigation::FootstepPlanTransformer bdi_to_world_transformer;
};
}

#endif
