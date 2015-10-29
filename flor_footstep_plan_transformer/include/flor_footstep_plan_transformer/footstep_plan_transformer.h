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

#ifndef FOOTSTEP_PLAN_TRANSFORMER_H__
#define FOOTSTEP_PLAN_TRANSFORMER_H__

#include <tf/tf.h>
#include <ros/time.h>

#include <flor_footstep_planner_msgs/flor_footstep_planner_msgs.h>
#include <flor_footstep_planner_msgs/StepTarget.h>
#include <flor_footstep_planner_msgs/FeetPoses.h>

namespace flor_navigation
{
/**
 * Transforms footstep plans from one frame to another
 */
class FootstepPlanTransformer
{
public:
  FootstepPlanTransformer();

  // setter and getter for transformation
  void setTransform(const geometry_msgs::PoseStamped& transform_in);
  const tf::StampedTransform& getTransform() const { return transform_world_to_bdi; }

  // generic interface for transforming plans
  template <typename T> inline bool transformPlanToWorld(T& plan) const { return transformPlan(plan, transform_bdi_to_world); }
  template <typename T> inline bool transformPlanToBdi(T& plan) const { return transformPlan(plan, transform_world_to_bdi); }

  // interface for transforming yaw angle
  inline void transformYawToWorld(float& yaw) const { transformYaw(yaw, transform_bdi_to_world); }
  inline void transformYawToBdi(float& yaw) const { transformYaw(yaw, transform_world_to_bdi); }

  // generic interface for transformation
  template <typename T> inline void transformToWorld(std::vector<T>& data) const
  {
    for (typename std::vector<T>::iterator itr = data.begin(); itr != data.end(); itr++)
      transform(*itr, transform_bdi_to_world);
  }
  template <typename T> inline void transformToBdi(std::vector<T>& data) const
  {
    for (typename std::vector<T>::iterator itr = data.begin(); itr != data.end(); itr++)
      transform(*itr, transform_world_to_bdi);
  }
  template <typename T> inline void transformToWorld(T& data) const { transform(data, transform_bdi_to_world); }
  template <typename T> inline void transformToBdi(T& data) const { transform(data, transform_world_to_bdi); }

protected:
  // generic implementation for transform plans
  template <typename T> bool transformPlan(T& plan, const tf::StampedTransform& transf) const
  {
    transform(plan.header, transf);
    for (size_t i = 0; i < plan.step_plan.size(); i++)
      transform(plan.step_plan[i], transf);
    return true;
  }

  // concrete implementations of transformations
  template <typename T> inline void transform(std::vector<T>& data, const tf::StampedTransform& transf) const
  {
    for (typename std::vector<T>::iterator itr = data.begin(); itr != data.end(); itr++)
      transform(*itr, transf);
  }

  void transformYaw(float& yaw, const tf::StampedTransform& transf) const;

  void transform(flor_footstep_planner_msgs::FootstepPlan& data, const tf::StampedTransform& transf) const;
  void transform(flor_footstep_planner_msgs::StepTarget& step, const tf::StampedTransform& transf) const;
  void transform(flor_footstep_planner_msgs::FeetPoses& feet_poses, const tf::StampedTransform& transf) const;
  void transform(std_msgs::Header& header, const tf::StampedTransform& transf) const;
  void transform(geometry_msgs::Vector3& vec, const tf::StampedTransform& transf) const;
  void transform(geometry_msgs::Point& point, const tf::StampedTransform& transf) const;
  void transform(geometry_msgs::Pose& pose, const tf::StampedTransform& transf) const;


  bool identity_transform_;
  tf::StampedTransform transform_world_to_bdi;
  tf::StampedTransform transform_bdi_to_world;
};



void addOriginShift(geometry_msgs::Point& point, double yaw, const geometry_msgs::Vector3& shift);
void addOriginShift(geometry_msgs::Pose& pose, const geometry_msgs::Vector3 &shift);
void addOriginShift(flor_footstep_planner_msgs::StepTarget& step, const geometry_msgs::Vector3 &shift);
void addOriginShift(flor_footstep_planner_msgs::FeetPoses& feet, const geometry_msgs::Vector3& shift);

void removeOriginShift(geometry_msgs::Point& point, double yaw, const geometry_msgs::Vector3& shift);
void removeOriginShift(geometry_msgs::Pose& pose, const geometry_msgs::Vector3& shift);
void removeOriginShift(flor_footstep_planner_msgs::StepTarget& step, const geometry_msgs::Vector3& shift);
void removeOriginShift(flor_footstep_planner_msgs::FeetPoses& feet, const geometry_msgs::Vector3& shift);

}

#endif
