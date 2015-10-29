// SVN $HeadURL: http://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_stacks/humanoid_navigation/footstep_planner/src/Heuristic.cpp $
// SVN $Id: Heuristic.cpp 3298 2012-09-28 11:37:38Z hornunga@informatik.uni-freiburg.de $

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

#include <footstep_planner/heuristics/Heuristic.h>

namespace footstep_planner
{
Heuristic::Heuristic(double cell_size, int num_angle_bins,
                     HeuristicType type)
: ivCellSize(cell_size),
  ivNumAngleBins(num_angle_bins),
  ivAngleBinSize(2.0*M_PI / ivNumAngleBins),
  ivHeuristicType(type)
{}


Heuristic::~Heuristic()
{}


EuclideanHeuristic::EuclideanHeuristic(double cell_size, int num_angle_bins)
: Heuristic(cell_size, num_angle_bins, EUCLIDEAN_HEURISTIC)
{}


EuclideanHeuristic::~EuclideanHeuristic()
{}


double
EuclideanHeuristic::getHValue(const PlanningState& from, const PlanningState& to)
const
{
  if (from == to)
    return 0.0;

  // distance in cell size
  return euclidean_distance(from.getState().getX(), from.getState().getY(), from.getState().getZ(), to.getState().getX(), to.getState().getY(), to.getState().getZ());
}


EuclStepCostHeuristic::EuclStepCostHeuristic(double cell_size,
                                             int    num_angle_bins,
                                             double step_cost,
                                             double diff_angle_cost,
                                             double max_step_dist_x,
                                             double max_step_dist_y)
: Heuristic(cell_size, num_angle_bins, EUCLIDEAN_STEP_COST_HEURISTIC),
  ivStepCost(step_cost),
  ivDiffAngleCost(diff_angle_cost),
  ivMaxStepDistX(max_step_dist_x),
  ivMaxStepDistY(max_step_dist_y)
{}


EuclStepCostHeuristic::~EuclStepCostHeuristic()
{}


double
EuclStepCostHeuristic::getHValue(const PlanningState& from, const PlanningState& to)
const
{
  if (from == to)
    return 0.0;

  // distance in meter
  double dist = euclidean_distance(from.getState().getX(), from.getState().getY(), from.getState().getZ(), to.getState().getX(), to.getState().getY(), to.getState().getZ());

  if (dist < ivMaxStepDistX || dist < ivMaxStepDistY)
    return dist;

  // expected steps
  tf::Transform step = from.getState().getPose().inverse() * to.getState().getPose();
  double expected_steps = std::floor(std::abs(step.getOrigin().x()) / ivMaxStepDistX + std::abs(step.getOrigin().y()) / ivMaxStepDistY);

//  ROS_INFO("%f %f %f", step.getOrigin().x(), ivMaxStepDistX, std::abs(step.getOrigin().x()) / ivMaxStepDistX);
//  ROS_INFO("%f %f %f", step.getOrigin().y(), ivMaxStepDistY, (std::abs(step.getOrigin().y()) / ivMaxStepDistY));
//  ROS_INFO("%f", expected_steps);

  double diff_angle = 0.0;
  if (ivDiffAngleCost > 0.0)
    diff_angle = std::abs(angles::shortest_angular_distance(to.getYaw(), from.getYaw()));

  return (dist + expected_steps * ivStepCost + diff_angle * ivDiffAngleCost);
}


GPRStepCostHeuristic::GPRStepCostHeuristic(double cell_size,
                                           int    num_angle_bins,
                                           double step_cost,
                                           double diff_angle_cost,
                                           double max_step_width)
: Heuristic(cell_size, num_angle_bins, GPR_STEP_COST_HEURISTIC),
  ivStepCost(step_cost),
  ivDiffAngleCost(diff_angle_cost),
  ivMaxStepWidth(max_step_width)
{}


GPRStepCostHeuristic::~GPRStepCostHeuristic()
{}


double
GPRStepCostHeuristic::getHValue(const PlanningState& from, const PlanningState& to)
const
{
  if (from == to)
    return 0.0;

  // distance in meter
  double dist = cont_val(euclidean_distance(from.getX(), from.getY(), to.getX(), to.getY()), ivCellSize);
  double expected_steps = dist / ivMaxStepWidth;
  double diff_angle = 0.0;
  if (ivDiffAngleCost > 0.0)
  {
    // get the number of bins between from.theta and to.theta
    int diff_angle_disc = (((to.getYaw() - from.getYaw()) % ivNumAngleBins) + ivNumAngleBins) % ivNumAngleBins;
    // get the rotation independent from the rotation direction
    diff_angle = std::abs(angles::normalize_angle(angle_cell_2_state(diff_angle_disc, ivAngleBinSize)));
  }

  return (dist + expected_steps * ivStepCost + diff_angle * ivDiffAngleCost);
}
}
