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

#ifndef FLOR_FOOTSTEP_PLANNER_VIS_H__
#define FLOR_FOOTSTEP_PLANNER_VIS_H__

#include <map>
#include <string>

#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include <flor_footstep_plan_transformer/footstep_plan_transformer.h>
#include <flor_footstep_planner_msgs/flor_footstep_planner_msgs.h>


namespace flor_footstep_plan_vis
{
class FootstepPlannerVis
{
public:
  FootstepPlannerVis(ros::NodeHandle &nh);
  virtual ~FootstepPlannerVis();

  void setFootSize(const geometry_msgs::Vector3 &size) { foot_size = size; }
  void setFootOriginShift(const geometry_msgs::Vector3 &shift) { foot_origin_shift = shift; }
  void setFootSeperation(double foot_seperation) { this->foot_seperation = foot_seperation; }
  void setUpperBodySize(const geometry_msgs::Vector3 &size) { upper_body_size = size; }
  void setUpperBodyOriginShift(const geometry_msgs::Vector3 &shift) { upper_body_origin_shift = shift; }

  void publishFeetStartPosesVis(ros::Publisher &pub, const flor_footstep_planner_msgs::FeetPoses &feetPoses);

  template <typename T> void publishFootstepPlanVis(ros::Publisher &pub, const T &plan)
  {
    if (pub.getNumSubscribers() > 0)
    {
      // clear old MarkerArrays
      for (visualization_msgs::MarkerArray::_markers_type::iterator itr = last_footstep_plan_marker_array.markers.begin(); itr != last_footstep_plan_marker_array.markers.end(); itr++)
      {
        visualization_msgs::Marker &marker(*itr);
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::DELETE;
      }
      pub.publish(last_footstep_plan_marker_array);

      // convert footstep plan to marker array
      footstepPlanToFootMarkerArray(plan.step_plan, last_footstep_plan_marker_array);
      pub.publish(last_footstep_plan_marker_array);
    }
  }

  template <typename T> void publishPathVis(ros::Publisher &pub, const T &plan)
  {
    if (pub.getNumSubscribers() > 0)
    {
      if (plan.step_plan.size() == 0)
      {
        ROS_INFO("no path has been extracted yet");
        return;
      }

      nav_msgs::Path path_msg;
      path_msg.header = plan.header;
      path_msg.header.stamp = ros::Time::now();

      for (size_t i = 0; i < plan.step_plan.size(); i++)
      {
        geometry_msgs::PoseStamped pose;
        pose.header = plan.step_plan[i].foot.header;
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = plan.step_plan[i].foot.position.x;
        pose.pose.position.y = plan.step_plan[i].foot.position.y;
        pose.pose.position.z = plan.step_plan[i].foot.position.z;

        path_msg.poses.push_back(pose);
      }

      pub.publish(path_msg);
    }
  }

  template <typename T> void publishFootstepPlanBodyVis(ros::Publisher &pub, const T &plan)
  {
    if (pub.getNumSubscribers() > 0)
    {
      // clear old MarkerArrays
      for (visualization_msgs::MarkerArray::_markers_type::iterator itr = last_footstep_plan_body_marker_array.markers.begin(); itr != last_footstep_plan_body_marker_array.markers.end(); itr++)
      {
        visualization_msgs::Marker &marker(*itr);
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::DELETE;
      }
      pub.publish(last_footstep_plan_body_marker_array);

      // convert footstep plan to marker array
      footstepPlanToBodyMarkerArray(plan.step_plan, last_footstep_plan_body_marker_array);
      pub.publish(last_footstep_plan_body_marker_array);
    }
  }

  template <typename T> void publishFootstepLogVis(ros::Publisher &pub, const std::map<unsigned int, T> &plan)
  {
    if (pub.getNumSubscribers() > 0)
    {
      // clear old MarkerArrays
      for (visualization_msgs::MarkerArray::_markers_type::iterator itr = last_footstep_log_marker_array.markers.begin(); itr != last_footstep_log_marker_array.markers.end(); itr++)
      {
        visualization_msgs::Marker &marker(*itr);
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::DELETE;
      }
      pub.publish(last_footstep_log_marker_array);

      // convert footstep plan to marker array
      footstepLogToFootMarkerArray(plan, last_footstep_log_marker_array);
      pub.publish(last_footstep_log_marker_array);
    }
  }

  // typedefs
  typedef boost::shared_ptr<FootstepPlannerVis> Ptr;
  typedef boost::shared_ptr<const FootstepPlannerVis> ConstPtr;

protected:
  // conversion functions
  template <typename T> void footstepPlanToFootMarkerArray(const std::vector<T> &plan, visualization_msgs::MarkerArray &markerArray) const
  {
    markerArray.markers.clear();
    for (typename std::vector<T>::const_iterator itr = plan.begin(); itr != plan.end(); itr++)
    {
      visualization_msgs::Marker marker;
      stepToMarker(*itr, marker);

      marker.id = markerArray.markers.size();
      //marker.color.r = itr->foot_index == flor_atlas_msgs::AtlasBehaviorFootData::FOOT_LEFT ? 1.0 : 0.0;
      //marker.color.g = itr->foot_index == flor_atlas_msgs::AtlasBehaviorFootData::FOOT_LEFT ? 0.0 : 1.0;
      marker.color.b = 0.0;
      marker.color.a = 0.6;
      markerArray.markers.push_back(marker);

      // add text
      marker.id++;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.text = boost::lexical_cast<std::string>(itr->step_index);
      marker.scale.z *= 3;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.color.a = 0.7;
      markerArray.markers.push_back(marker);
    }
  }

  template <typename T> void footstepLogToFootMarkerArray(const std::map<unsigned int, T> &plan, visualization_msgs::MarkerArray &markerArray) const
  {
    markerArray.markers.clear();
    for (typename std::map<unsigned int, T>::const_iterator itr = plan.begin(); itr != plan.end(); itr++)
    {
      visualization_msgs::Marker marker;
      stepToMarker(itr->second, marker);

      marker.id = markerArray.markers.size();
      // make them a little bit smaller
      marker.scale.x *= 0.95;
      marker.scale.y *= 0.95;
      marker.scale.z *= 0.95;
      marker.color.r = 0.5;
      marker.color.g = 0.0;
      marker.color.b = 0.5;
      marker.color.a = 0.65;
      markerArray.markers.push_back(marker);

      // add text
      marker.id++;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.text = boost::lexical_cast<std::string>(itr->second.step_index);
      marker.scale.z *= 3;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.color.a = 0.7;
      markerArray.markers.push_back(marker);
    }
  }

  template <typename T> void stepToMarker(const T &step, visualization_msgs::Marker &marker) const
  {
    T step_transformed = step;
    step_transformed.foot.position.z += foot_size.z/2; // marker should touch ground

    // shift to foot center (remove shift to foot frame)
    flor_navigation::removeOriginShift(step_transformed, foot_origin_shift);

    marker.header = step_transformed.foot.header;
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // compute absolut position of foot
    flor_footstep_planner_msgs::copyPosition(step_transformed.foot.position, marker.pose.position);
    flor_footstep_planner_msgs::normalToQuaternion(step_transformed.foot.yaw, step_transformed.foot.normal, marker.pose.orientation);

    // rescale marker based on foot size
    marker.scale = foot_size;
  }

  template <typename T> void footstepPlanToBodyMarkerArray(const std::vector<T> &plan, visualization_msgs::MarkerArray &markerArray) const
  {
    if (plan.empty())
      return;

    typename std::vector<T>::const_iterator itr = plan.begin();
    T last_step = *itr;

    visualization_msgs::Marker marker;
    marker.header = last_step.foot.header;
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    markerArray.markers.clear();
    for (itr++; itr != plan.end(); itr++)
    {
      T current_step = *itr;

      // approximate upper body dimensions
      float x = current_step.foot.position.x + 0.5 * (last_step.foot.position.x - current_step.foot.position.x);
      float y = current_step.foot.position.y + 0.5 * (last_step.foot.position.y - current_step.foot.position.y);
      float z = current_step.foot.position.z + 0.5 * (last_step.foot.position.z - current_step.foot.position.z);
      float theta = current_step.foot.yaw + 0.5 * (last_step.foot.yaw - current_step.foot.yaw);

      // compute center position of body
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = z;
      marker.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

      // determine shift of polygon based on orientation
      flor_navigation::addOriginShift(marker.pose, upper_body_origin_shift);

      // rescale marker based on body size
      marker.scale.x = upper_body_size.x;
      marker.scale.y = upper_body_size.y;
      marker.scale.z = 0.05;

      marker.id = markerArray.markers.size();
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 0.5;
      marker.color.a = 0.2;
      markerArray.markers.push_back(marker);

      last_step = current_step;
    }
  }

  visualization_msgs::MarkerArray last_footstep_plan_marker_array;
  visualization_msgs::MarkerArray last_foot_pose_start_marker_array;
  visualization_msgs::MarkerArray last_footstep_plan_body_marker_array;
  visualization_msgs::MarkerArray last_footstep_log_marker_array;

  // Parameters
  geometry_msgs::Vector3 foot_size;
  geometry_msgs::Vector3 foot_origin_shift;
  double foot_seperation;

  geometry_msgs::Vector3 upper_body_size;
  geometry_msgs::Vector3 upper_body_origin_shift;
};
}

#endif
