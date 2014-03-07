// SVN $HeadURL: http://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_stacks/humanoid_navigation/footstep_planner/src/FootstepPlannerNode.cpp $
// SVN $Id: FootstepPlannerNode.cpp 3298 2012-09-28 11:37:38Z hornunga@informatik.uni-freiburg.de $

/*
 * A footstep planner for humanoid robots
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

#include <footstep_planner/FootstepPlannerNode.h>

namespace footstep_planner
{
FootstepPlannerNode::FootstepPlannerNode()
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");

  // provide callbacks to interact with the footstep planner:
  ivGroundLevelGridMapSub = nh.subscribe<nav_msgs::OccupancyGrid>("ground_level_grid_map", 1, &FootstepPlanner::groundLevelMapCallback, &ivFootstepPlanner);
  ivBaseLevelGridMapSub = nh.subscribe<nav_msgs::OccupancyGrid>("body_level_grid_map", 1, &FootstepPlanner::bodyLevelMapCallback, &ivFootstepPlanner);
  ivGoalPoseSub = nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &FootstepPlanner::goalPoseCallback, &ivFootstepPlanner);
  ivStartPoseSub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, &FootstepPlanner::startPoseCallback, &ivFootstepPlanner);
  ivTerrainModelSub = nh.subscribe<flor_terrain_classifier::TerrainModel>("/flor/terrain_classifier/terrain_model", 1, &FootstepPlanner::terrainModelCallback, &ivFootstepPlanner);

  // service:
  ivSetParamsService = nh_.advertiseService("set_params", &FootstepPlanner::setParamsService, &ivFootstepPlanner);
  ivFootstepPlanService = nh.advertiseService("plan_footsteps", &FootstepPlanner::planFootstepService, &ivFootstepPlanner);
  ivSteppingPlanService = nh.advertiseService("plan_stepping", &FootstepPlanner::planSteppingService, &ivFootstepPlanner);
}


FootstepPlannerNode::~FootstepPlannerNode()
{}
}
