//=================================================================================================
// Copyright (c) 2013, Stefan Kohlbrecher, TU Darmstadt
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

#include <ros/ros.h>

#include <flor_footstep_planner_msgs/AtlasWalkFootstepPlan.h>
#include <flor_footstep_planner_msgs/AtlasStepFootstepPlan.h>
#include <flor_footstep_planner_msgs/FeetPoses.h>

#include <flor_footstep_plan_transformer/footstep_plan_transformer.h>


namespace flor_navigation
{
class FootStepPlanTransformerNode
{
public:
  FootStepPlanTransformerNode(ros::NodeHandle& nh_in)
  {
    transform_pose_sub_ = nh_in.subscribe("/flor/state/bdi_to_world_pose", 1, &FootstepPlanTransformer::setTransform, &transformer_);
    orig_plan_walk_sub_ = nh_in.subscribe("/flor/planner/footstep_plan_walk_world", 1, &FootStepPlanTransformerNode::origPlanWalkCb, this);
    orig_plan_step_sub_ = nh_in.subscribe("/flor/planner/footstep_plan_step_world", 1, &FootStepPlanTransformerNode::origPlanStepCb, this);

    //transformed_plan_walk_pub_ = nh_in.advertise<flor_footstep_planner_msgs::AtlasWalkFootstepPlan>("/flor/controller/footstep_plan_walk", 5, false);
    //transformed_plan_step_pub_ = nh_in.advertise<flor_footstep_planner_msgs::AtlasStepFootstepPlan>("/flor/controller/footstep_plan_step", 5, false);
  }

  void origPlanWalkCb(const flor_footstep_planner_msgs::AtlasWalkFootstepPlan::ConstPtr& msg)
  {
    flor_footstep_planner_msgs::AtlasWalkFootstepPlan transformed_plan = *msg;

    //if (transformer_.transformPlanToBdi(transformed_plan))
    //  transformed_plan_walk_pub_.publish(transformed_plan);
    //else
    //  ROS_ERROR("Unable to transform walk plan to BDI frame, not sending to controller");
  }

  void origPlanStepCb(const flor_footstep_planner_msgs::AtlasStepFootstepPlan::ConstPtr& msg)
  {
    flor_footstep_planner_msgs::AtlasStepFootstepPlan transformed_plan = *msg;

    //if (transformer_.transformPlanToBdi(transformed_plan))
    //  transformed_plan_step_pub_.publish(transformed_plan);
    //else
    //  ROS_ERROR("Unable to transform step plan to BDI frame, not sending to controller");
  }

protected:
  ros::Publisher transformed_plan_walk_pub_;
  ros::Publisher transformed_plan_step_pub_;

  ros::Subscriber transform_pose_sub_;
  ros::Subscriber orig_plan_walk_sub_;
  ros::Subscriber orig_plan_step_sub_;

  FootstepPlanTransformer transformer_;
};
}

int main (int argc, char **argv)
{
  ros::init(argc,argv,"footstep_plan_transformer_node");
  ros::NodeHandle nh;

  flor_navigation::FootStepPlanTransformerNode ft(nh);

  ros::spin();

  return 0;
}
