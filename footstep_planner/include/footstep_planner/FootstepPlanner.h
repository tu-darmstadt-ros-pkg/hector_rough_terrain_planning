// SVN $HeadURL: http://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_stacks/humanoid_navigation/footstep_planner/include/footstep_planner/FootstepPlanner.h $
// SVN $Id: FootstepPlanner.h 4170 2013-05-21 11:18:20Z garimort@informatik.uni-freiburg.de $

/*
 * A footstep planner for humanoid robots.
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/footstep_planner
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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

#ifndef FOOTSTEP_PLANNER_FOOTSTEPPLANNER_H_
#define FOOTSTEP_PLANNER_FOOTSTEPPLANNER_H_

#include <ros/ros.h>
#include <tf/tf.h>

#include <assert.h>
#include <time.h>

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <flor_atlas_msgs/AtlasBehaviorFootData.h>
#include <flor_atlas_msgs/AtlasBehaviorStepAction.h>
#include <flor_footstep_planner_msgs/FootstepPlannerParamsService.h>
#include <flor_footstep_planner_msgs/PlanFootsteps.h>
#include <flor_footstep_planner_msgs/PlanStepping.h>
#include <flor_footstep_planner_msgs/Stepping.h>
#include <flor_footstep_planner_msgs/flor_footstep_planner_msgs.h>
#include <flor_terrain_classifier/TerrainModelService.h>
#include <flor_footstep_plan_transformer/footstep_plan_transformer.h>

#include <footstep_planner/helper.h>
#include <footstep_planner/FootstepPlannerEnvironment.h>
#include <footstep_planner/PlanningStateChangeQuery.h>
#include <footstep_planner/State.h>
#include <footstep_planner/Footstep.h>


namespace footstep_planner
{
typedef std::vector<State>::const_iterator state_iter_t;

/**
 * @brief A class to control the interaction between ROS and the footstep
 * planner.
 */
class FootstepPlanner
{
public:
  FootstepPlanner();
  virtual ~FootstepPlanner();

  bool setParamsService(flor_footstep_planner_msgs::FootstepPlannerParamsService::Request &req, flor_footstep_planner_msgs::FootstepPlannerParamsService::Response &resp);

  /**
   * @brief Start a planning task from scratch (will delete information
   * of previous planning tasks). Map and start, goal poses need to be
   * set beforehand.
   *
   * @return Success of planning.
   */
  bool plan(uint32_t &status_flags);

  /// @brief Sets start, goal poses and calls FootstepPlanner::plan().
  bool plan(const geometry_msgs::PoseStampedConstPtr start,
            const geometry_msgs::PoseStampedConstPtr goal);

  /// @brief Sets start, goal poses and calls FootstepPlanner::plan().
  bool plan(float start_x, float start_y, float start_theta,
            float goal_x, float goal_y, float goal_theta);

  /// @brief Sets start, goal poses and calls FootstepPlanner::plan().
  bool plan(flor_footstep_planner_msgs::PlanFootsteps::Request &req, flor_footstep_planner_msgs::PlanFootsteps::Response &resp);

  /// @brief plans stepping
  bool planStepping(flor_footstep_planner_msgs::PlanStepping::Request &req, flor_footstep_planner_msgs::PlanStepping::Response &resp);
  bool addStepping(const flor_footstep_planner_msgs::PlanStepping::Request &req, State &s, bool change_z = true);

  /// @brief plans realignment of feet
  bool planRealignFeet(const flor_footstep_planner_msgs::PlanStepping::Request &req);

  /**
   * @brief Starts a planning task based on previous planning information
   * (note that this method can also be used when no previous planning was
   * performed). Map and start, goal poses need to be set beforehand.
   *
   * @return Success of planning.
   */
  bool replan(uint32_t &status_flags);

  /// @brief Service handle to plan footsteps.
  bool planFootstepService(flor_footstep_planner_msgs::PlanFootsteps::Request &req,
                           flor_footstep_planner_msgs::PlanFootsteps::Response &resp);

  /// @brief Service handle to plan stepping.
  bool planSteppingService(flor_footstep_planner_msgs::PlanStepping::Request &req,
                           flor_footstep_planner_msgs::PlanStepping::Response &resp);

  /// @brief generic method which generates response
  template <typename Treq, typename Tresp> bool planService(Treq &/*req*/, Tresp &resp)
  {
    resp.footstep_plan.header.frame_id = frame_id;
    resp.footstep_plan.header.stamp = ros::Time::now();

    // set feet start poses
    resp.feet_start_poses.header = resp.footstep_plan.header;

    resp.feet_start_poses.left.header = resp.footstep_plan.header;
    ivStartFootLeft.getFootData(resp.feet_start_poses.left);

    resp.feet_start_poses.right.header = resp.footstep_plan.header;
    ivStartFootRight.getFootData(resp.feet_start_poses.right);

    // add shift to foot frame
    flor_navigation::addOriginShift(resp.feet_start_poses, ivEnvironmentParams.foot_origin_shift);

    // set planning mode
    resp.footstep_plan.planning_mode = ivPlannerEnvironmentPtr->getPlanningMode();

    // add footstep plan
    flor_footstep_planner_msgs::StepTarget step;
    step.header = resp.footstep_plan.header;
    step.foot.header = resp.footstep_plan.header;

    State left_foot = ivStartFootLeft;
    State right_foot = ivStartFootRight;

    int step_index = 0;
    resp.footstep_plan.step_plan.reserve(getPathSize());
    for (state_iter_t path_iter = getPathBegin(); path_iter != getPathEnd(); ++path_iter)
    {
      const State& swing_foot = *path_iter;
      const State& stand_foot = swing_foot.getLeg() == LEFT ? right_foot : left_foot;

      // convert footstep
      swing_foot.getStep(step);
      if (swing_foot.getLeg() == LEFT)
        step.foot_index = flor_atlas_msgs::AtlasBehaviorFootData::FOOT_LEFT;
      else if (swing_foot.getLeg() == RIGHT)
        step.foot_index = flor_atlas_msgs::AtlasBehaviorFootData::FOOT_RIGHT;
      else
      {
        ROS_ERROR("Footstep pose at (%f, %f, %f, %f) is set to NOLEG!",
                  swing_foot.getX(), swing_foot.getY(), swing_foot.getZ(), swing_foot.getYaw());
        continue;
      }
      step.step_index = step_index++;

      // add step specific parameters tweaking it
      postProcessStep(left_foot, right_foot, step);

      if (std::abs(stand_foot.getZ() - swing_foot.getZ()) > 0.18)
        resp.status_flags |= flor_footstep_planner_msgs::PlanFootstepsResponse::LARGE_HEIGHT_DIFF;

      // add shift to foot frame
      flor_navigation::addOriginShift(step, ivEnvironmentParams.foot_origin_shift);

      // add finally to plan
      resp.footstep_plan.step_plan.push_back(step);

      // some debug outputs and visualization stuff
      ROS_INFO("[%i] n: %f/%f/%f, z: %f", step.step_index, step.foot.normal.x, step.foot.normal.y, step.foot.normal.z, step.foot.position.z);
      ROS_INFO("[%i] step duration: %f, sway_duration: %f", step.step_index, step.step_duration, step.sway_duration);
      ROS_INFO("[%i] lift height: %f, swing height: %f", step.step_index, step.lift_height, step.swing_height);
      ROS_INFO("[%i] TOE-OFF: %i, knee nominal: %f", step.step_index, step.toe_off, step.knee_nominal);
      double risk = 0.0;
      double cost = ivPlannerEnvironmentPtr->getStepCostEstimator()->getCost(left_foot, right_foot, swing_foot, risk);
      ROS_INFO("[%i] cost: %f risk: %f", step.step_index, cost, risk);

      if (ivPlannerEnvironmentPtr->getTerrainModel())
      {
        double support = 0.0;
        ivPlannerEnvironmentPtr->getTerrainModel()->getFootContactSupport(swing_foot, support, ivCheckedFootContactSupport);
        ROS_INFO("[%i] Ground contact support: %f", step.step_index, support);
      }

      ROS_INFO("-------------------------------------");

      // next step
      if (swing_foot.getLeg() == LEFT)
        left_foot = swing_foot;
      else if (swing_foot.getLeg() == RIGHT)
        right_foot = swing_foot;
    }

    if (ivCheckedFootContactSupportPub.getNumSubscribers() > 0)
    {
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(*ivCheckedFootContactSupport, msg);
      msg.header.frame_id = frame_id;
      msg.header.stamp = ros::Time::now();
      ivCheckedFootContactSupportPub.publish(msg);
    }

    return true;
  }

  void postProcessStep(const State &left_foot, const State &right_foot, flor_footstep_planner_msgs::StepTarget &swing_foot) const;

  bool findNearestValidState(State &s) const;

  bool checkRobotCollision(const State &left_foot, const State &right_foot, bool &left, bool &right) const;

  /**
   * @brief Sets the goal pose
   *
   * @return True if the two foot poses have been set successfully.
   */
  template <typename T> bool setGoalFromReq(const T &req, bool ignore_collision = false)
  {
    // set start foot poses
    switch (req.goal_type)
    {
      case flor_footstep_planner_msgs::PlanFootsteps::Request::POSE:
      {
        if (!setGoal(req.goal.position.x, req.goal.position.y, tf::getYaw(req.goal.orientation), ignore_collision))
          return false;
        break;
      }
      case flor_footstep_planner_msgs::PlanFootsteps::Request::FEET:
      {
        State left_foot(req.feet_goal_poses.left.position,
                        req.feet_goal_poses.left.normal,
                        req.feet_goal_poses.left.yaw,
                        ivEnvironmentParams.swing_height, ivEnvironmentParams.lift_height,
                        ivEnvironmentParams.step_duration, ivEnvironmentParams.sway_duration,
                        LEFT);

        State right_foot(req.feet_goal_poses.right.position,
                         req.feet_goal_poses.right.normal,
                         req.feet_goal_poses.right.yaw,
                         ivEnvironmentParams.swing_height, ivEnvironmentParams.lift_height,
                         ivEnvironmentParams.step_duration, ivEnvironmentParams.sway_duration,
                         RIGHT);

        if (!setGoal(left_foot, right_foot, ignore_collision))
          return false;
        break;
      }
    }

    return true;
  }

  /**
   * @brief Sets the goal pose as a robot pose centered between two feet.
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setGoal(const geometry_msgs::PoseStampedConstPtr goal_pose, bool ignore_collision = false);

  /**
   * @brief Sets the goal pose as a robot pose centered between two feet.
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setGoal(float x, float y, float yaw, bool ignore_collision = false);

  /**
   * @brief Sets the goal pose as position of left and right footsteps.
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setGoal(const State& left_foot, const State& right_foot, bool ignore_collision = false);

  /**
   * @brief Sets the start pose
   *
   * @return True if the two foot poses have been set successfully.
   */
  template <typename T> bool setStartFromReq(const T &req, bool ignore_collision = false)
  {
    // set start foot poses
    switch (req.start_type)
    {
      case flor_footstep_planner_msgs::PlanFootsteps::Request::POSE:
      {
        if (!setStart(req.start.position.x, req.start.position.y, tf::getYaw(req.start.orientation), ignore_collision))
          return false;
        break;
      }
      case flor_footstep_planner_msgs::PlanFootsteps::Request::FEET:
      {
        flor_footstep_planner_msgs::FeetPoses feet_poses = req.feet_start_poses;

        // remove shift to foot frame
        flor_navigation::removeOriginShift(feet_poses, ivEnvironmentParams.foot_origin_shift);

        State left_foot(feet_poses.left.position,
                        feet_poses.left.normal,
                        feet_poses.left.yaw,
                        ivEnvironmentParams.swing_height, ivEnvironmentParams.lift_height,
                        ivEnvironmentParams.step_duration, ivEnvironmentParams.sway_duration,
                        LEFT);

        State right_foot(feet_poses.right.position,
                         feet_poses.right.normal,
                         feet_poses.right.yaw,
                         ivEnvironmentParams.swing_height, ivEnvironmentParams.lift_height,
                         ivEnvironmentParams.step_duration, ivEnvironmentParams.sway_duration,
                         RIGHT);

        if (!setStart(left_foot, right_foot, ignore_collision))
          return false;
        break;
      }
    }

    return true;
  }

  /**
   * @brief Sets the start pose as a robot pose centered between two feet.
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setStart(const geometry_msgs::PoseStampedConstPtr start_pose, bool ignore_collision = false);

  /**
   * @brief Sets the start pose as a robot pose centered between two feet.
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setStart(float x, float y, float yaw, bool ignore_collision = false);

  /**
   * @brief Sets the start pose as position of left and right footsteps.
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setStart(const State& left_foot, const State& right_foot, bool ignore_collision = false);

  /**
   * @brief Updates the map in the planning environment.
   *
   * @return True if a replanning is necessary, i.e. the old path is not valid
   * any more.
   */
  bool updateGroundLevelMap(const gridmap_2d::GridMap2DPtr& map);
  bool updateBodyLevelMap(const gridmap_2d::GridMap2DPtr& map);

  void setMarkerNamespace(const std::string& ns) { ivMarkerNamespace = ns; }

  /// @brief Set the maximal search time.
  void setMaxSearchTime(int search_time) { ivMaxSearchTime = search_time; }

  /**
   * @brief Callback to set the goal pose as a robot pose centered between
   * two feet. If the start pose has been set previously the planning is
   * started.
   *
   * Subscribed to 'goal'.
   *
   * @return True if the two foot poses have been set successfully.
   */
  void goalPoseCallback(
      const geometry_msgs::PoseStampedConstPtr& goal_pose);
  /**
   * @brief Callback to set the start pose as a robot pose centered
   * between two feet. If the goal pose has been set previously the
   * planning is started.
   *
   * Subscribed to 'initialpose'.
   *
   * @return True if the two foot poses have been set successfully.
   */
  void startPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& start_pose);

  /**
   * @brief Callback to set the map.
   *
   * Subscribed to 'map'.
   */
  void groundLevelMapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_map);
  void bodyLevelMapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_map);

  /**
   * @brief Callback to set the terrain model.
   *
   * Subscribed to '/flor/terrain_classifier/terrain_model'.
   */
  void terrainModelCallback(const flor_terrain_classifier::TerrainModelConstPtr& terrain_model);

  /**
   * @brief Clear the footstep path visualization from a previous planning
   * task.
   */
  void clearFootstepPathVis(unsigned num_footsteps=0);

  /// @return Costs of the planned footstep path.
  double getPathCosts() const { return ivPathCost; }

  /// @return Number of expanded states.
  size_t getNumExpandedStates() const
  {
    return ivPlannerPtr->get_n_expands();
  }

  /// @return Number of planned foot poses.
  size_t getNumFootPoses() const { return ivPath.size(); }

  state_iter_t getPathBegin() const { return ivPath.begin(); }
  state_iter_t getPathEnd() const { return ivPath.end(); }

  /// @return Size of the planned path.
  int getPathSize() { return ivPath.size(); }

  State getStartFootLeft() { return ivStartFootLeft; }
  State getStartFootRight() { return ivStartFootRight; }

  /// @brief Reset the previous planning information.
  void reset();

  /// @brief Reset and reinitialize the environment.
  void resetTotally();

  /// @return True if for the current start and goal pose a path exists.
  bool pathExists() { return (bool)ivPath.size(); }

  /// @brief Planning parameters.
  environment_params ivEnvironmentParams;

protected:
  void broadcastExpandedNodesVis();
  void broadcastRandomNodesVis();
  void broadcastFootstepPathVis();
  void broadcastHeuristicPathVis();
  void broadcastPathVis();
  void publishParams();

  /**
   * @return True if the newly calculated path is different from the existing
   * one (if one exists).
   */
  bool pathIsNew(const std::vector<int>& new_path);

  /**
   * @brief Extracts the path (list of foot poses) from a list of state
   * IDs calculated by the SBPL.
   */
  bool extractPath(const std::vector<int>& state_ids);

  /// @brief Generates a visualization msgs for a foot pose.
  void footPoseToMarker(const State& footstep,
                        visualization_msgs::Marker* marker);

  /**
   * @brief Starts the planning task in the underlying SBPL.
   *
   * NOTE: Never call this directly. Always use either plan() or replan() to
   * invoke this method.
   */
  bool run();

  /// @brief Returns the foot pose of a leg for a given robot pose.
  State getFootPose(const State& robot, Leg leg, bool use_terrain_model);
  State getFootPose(const State& robot, Leg leg, double dx, double dy, double dyaw, bool use_terrain_model);

  /// @brief get parallel foot
  State getParallelFootPose(const State& foot, bool use_terrain_model, double additional_seperation = 0.0);

  /// @brief Sets the planning algorithm used by SBPL.
  void setPlanner();

  /// @brief Updates the environment in case of a changed map.
  void updateEnvironment(const gridmap_2d::GridMap2DPtr old_map);

  boost::shared_ptr<FootstepPlannerEnvironment> ivPlannerEnvironmentPtr;
  gridmap_2d::GridMap2DPtr ivGroundLevelMapPtr;
  gridmap_2d::GridMap2DPtr ivBodyLevelMapPtr;
  std::string frame_id;
  boost::shared_ptr<SBPLPlanner> ivPlannerPtr;
  flor_terrain_classifier::TerrainModel::ConstPtr ivTerrainModelPtr;

  boost::shared_ptr<const PathCostHeuristic> ivPathCostHeuristicPtr;

  std::vector<State> ivPath;

  State ivStartFootLeft;
  State ivStartFootRight;
  State ivGoalFootLeft;
  State ivGoalFootRight;

  ros::Publisher ivExpandedStatesVisPub;
  ros::Publisher ivFootstepPathVisPub;
  ros::Publisher ivRandomStatesVisPub;
  ros::Publisher ivHeuristicPathVisPub;
  ros::Publisher ivPathVisPub;
  ros::Publisher ivStartPoseVisPub;
  ros::Publisher ivCheckedFootContactSupportPub;
  ros::Publisher ivCurrentParamsPub;

  ros::ServiceClient ivTerrainModelService;


  // Parameters
  bool   ivStartPoseSetUp, ivGoalPoseSetUp;
  int    ivLastMarkerMsgSize;
  double ivPathCost;
  bool   ivSearchUntilFirstSolution;
  double ivMaxSearchTime;
  double ivInitialEpsilon;

  /**
   * @brief If limit of changed cells is reached the planner starts a new
   * task from the scratch.
   */
  int ivChangedCellsLimit;

  bool ivShiftGoalPose;

  std::string ivPlannerType;
  std::string ivMarkerNamespace;

  std::vector<int> ivPlanningStatesIds;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ivCheckedFootContactSupport;
};
}

#endif  // FOOTSTEP_PLANNER_FOOTSTEPPLANNER_H_
