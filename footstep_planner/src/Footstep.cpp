// SVN $HeadURL: https://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_navigation/footstep_planner/src/Dstar.cpp $
// SVN $Id: Dstar.cpp 1168 2011-03-30 03:18:02Z hornunga@informatik.uni-freiburg.de $

/*
 * A footstep planner for humanoid robots
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/footstep_planner
 *
 * D* Lite (Koenig et al. 2002) partly based on the implementation
 * by J. Neufeld (http://code.google.com/p/dstarlite/)
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

#include <footstep_planner/Footstep.h>


namespace footstep_planner
{
Footstep::Footstep(double x, double y, double theta, double swing_height, double lift_height, double step_duration, double sway_duration,
                   double step_cost, double cell_size, int num_angle_bins, int max_hash_size)
  : ivCellSize(cell_size)
  , ivNumAngleBins(num_angle_bins)
  , ivAngleBinSize(2.0*M_PI / ivNumAngleBins)
  , ivTheta(angle_state_2_cell(theta, ivAngleBinSize))
  , ivSwingHeight(swing_height)
  , ivLiftHeight(lift_height)
  , ivStepDuration(step_duration)
  , ivSwayDuration(sway_duration)
  , ivStepCost(step_cost)
  , ivMaxHashSize(max_hash_size)
  , ivDiscSuccessorLeft(num_angle_bins)
  , ivDiscSuccessorRight(num_angle_bins)
  , ivDiscPredecessorLeft(num_angle_bins)
  , ivDiscPredecessorRight(num_angle_bins)
{
  init(x, y);
}


Footstep::~Footstep()
{}


void
Footstep::init(double x, double y)
{
  int backward_angle;
  int footstep_x;
  int footstep_y;

  for (int a = 0; a < ivNumAngleBins; ++a)
  {
    backward_angle = calculateForwardStep(RIGHT, a, x, y, &footstep_x, &footstep_y);
    ivDiscSuccessorRight[a] = footstep_xy(footstep_x, footstep_y);
    ivDiscPredecessorLeft[backward_angle] = footstep_xy(-footstep_x, -footstep_y);

    backward_angle = calculateForwardStep(LEFT, a, x, y, &footstep_x, &footstep_y);
    ivDiscSuccessorLeft[a] = footstep_xy(footstep_x, footstep_y);
    ivDiscPredecessorRight[backward_angle] = footstep_xy(-footstep_x, -footstep_y);
  }
}


PlanningState
Footstep::performMeOnThisState(const PlanningState& current, const TerrainModel::ConstPtr &terrain_model)
const
{
  State state = current.getState();
  int x = current.getX();
  int y = current.getY();
  int yaw = current.getYaw();

  // theta has to be in [0..ivNumAngleBins)
  if (yaw < 0)
    yaw += ivNumAngleBins;
  else if (yaw >= ivNumAngleBins)
    yaw -= ivNumAngleBins;

  if (current.getLeg() == RIGHT)
  {
    footstep_xy xy = ivDiscSuccessorRight[yaw];
    x += xy.first;
    y += xy.second;
    yaw += ivTheta;
    state.setLeg(LEFT);
  }
  else // leg == LEFT
  {
    footstep_xy xy = ivDiscSuccessorLeft[yaw];
    x += xy.first;
    y += xy.second;
    yaw -= ivTheta;
    state.setLeg(RIGHT);
  }

  state.setX(cell_2_state(x, ivCellSize));
  state.setY(cell_2_state(y, ivCellSize));

  double yaw_d = angles::normalize_angle(angle_cell_2_state(yaw, ivAngleBinSize));
  state.setYaw(yaw_d);

  if (!terrain_model || !terrain_model->add3DData(state))
    state.setRPY(0.0, 0.0, yaw_d);

  return PlanningState(state, ivCellSize, ivAngleBinSize, ivMaxHashSize, &current, NULL);
}


PlanningState
Footstep::reverseMeOnThisState(const PlanningState& current, const TerrainModel::ConstPtr &terrain_model)
const
{
  State state = current.getState();
  int x = current.getX();
  int y = current.getY();
  int yaw = current.getYaw();

  // theta has to be in [0..ivNumAngleBins)
  if (yaw < 0)
    yaw += ivNumAngleBins;
  else if (yaw >= ivNumAngleBins)
    yaw -= ivNumAngleBins;

  if (current.getLeg() == RIGHT)
  {
    footstep_xy xy = ivDiscPredecessorRight[yaw];
    x += xy.first;
    y += xy.second;
    yaw += ivTheta;
    state.setLeg(LEFT);
  }
  else // leg == LEFT
  {
    footstep_xy xy = ivDiscPredecessorLeft[yaw];
    x += xy.first;
    y += xy.second;
    yaw -= ivTheta;
    state.setLeg(RIGHT);
  }

  double yaw_d = angles::normalize_angle(angle_cell_2_state(yaw, ivAngleBinSize));
  state.setYaw(yaw_d);

  if (!terrain_model || !terrain_model->add3DData(state))
    state.setRPY(0.0, 0.0, yaw_d);

  return PlanningState(state, ivCellSize, ivAngleBinSize, ivMaxHashSize, NULL, &current);
}


int
Footstep::calculateForwardStep(Leg leg, int global_theta,
                               double x, double y,
                               int* footstep_x, int* footstep_y)
const
{
  double cont_footstep_x, cont_footstep_y;
  double cont_global_theta = angle_cell_2_state(global_theta, ivAngleBinSize);
  double theta_cos = cos(cont_global_theta);
  double theta_sin = sin(cont_global_theta);
  if (leg == RIGHT)
  {
    cont_footstep_x = theta_cos * x - theta_sin * y;
    cont_footstep_y = theta_sin * x + theta_cos * y;

    global_theta += ivTheta;
  }
  else // leg == LEFT
  {
    cont_footstep_x = theta_cos * x + theta_sin * y;
    cont_footstep_y = theta_sin * x - theta_cos * y;

    global_theta -= ivTheta;
  }
  *footstep_x = disc_val(cont_footstep_x, ivCellSize);
  *footstep_y = disc_val(cont_footstep_y, ivCellSize);

  // theta has to be in [0..ivNumAngleBins)
  if (global_theta < 0)
    global_theta += ivNumAngleBins;
  else if (global_theta >= ivNumAngleBins)
    global_theta -= ivNumAngleBins;
  return global_theta;
}
} // end of namespace
