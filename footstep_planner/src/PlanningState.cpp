// SVN $HeadURL: http://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_stacks/humanoid_navigation/footstep_planner/src/PlanningState.cpp $
// SVN $Id: PlanningState.cpp 4146 2013-05-13 13:57:41Z garimort@informatik.uni-freiburg.de $

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

#include <footstep_planner/PlanningState.h>


namespace footstep_planner
{
PlanningState::PlanningState(double x, double y, double z, double roll, double pitch, double yaw, double swing_height, double lift_height, double step_duration, double sway_duration,
                             Leg leg, double cell_size, double angle_bin_size, int max_hash_size, const PlanningState *pred_state, const PlanningState *succ_state)
  : ivState(State(x, y, z, roll, pitch, yaw, swing_height, lift_height, step_duration, sway_duration, leg))
  , ivX(state_2_cell(x, cell_size))
  , ivY(state_2_cell(y, cell_size))
  , ivYaw(angle_state_2_cell(yaw, angle_bin_size))
  , ivpPredState(pred_state)
  , ivpSuccState(succ_state)
  , ivId(-1)
  , ivHashTag(calc_hash_tag(ivX, ivY, ivYaw, leg, max_hash_size))
{}


PlanningState::PlanningState(int x, int y, double z, double roll, double pitch, int yaw, double swing_height, double lift_height, double step_duration, double sway_duration,
                             Leg leg, double cell_size, double angle_bin_size, int max_hash_size, const PlanningState *pred_state, const PlanningState *succ_state)
  : ivX(x)
  , ivY(y)
  , ivYaw(yaw)
  , ivpPredState(pred_state)
  , ivpSuccState(succ_state)
  , ivId(-1)
  , ivHashTag(calc_hash_tag(ivX, ivY, ivYaw, leg, max_hash_size))
{
  ivState = State(cell_2_state(ivX, cell_size),
                  cell_2_state(ivY, cell_size),
                  z,
                  roll,
                  pitch,
                  angles::normalize_angle(angle_cell_2_state(ivYaw, angle_bin_size)),
                  swing_height,
                  lift_height,
                  step_duration,
                  sway_duration,
                  leg);
}

PlanningState::PlanningState(const geometry_msgs::Pose &pose, double swing_height, double lift_height, double step_duration, double sway_duration,
                             Leg leg, double cell_size, double angle_bin_size, int max_hash_size, const PlanningState *pred_state, const PlanningState *succ_state)
  : ivState(State(pose, swing_height, lift_height, step_duration, sway_duration, leg))
  , ivX(state_2_cell(pose.position.x, cell_size))
  , ivY(state_2_cell(pose.position.y, cell_size))
  , ivpPredState(pred_state)
  , ivpSuccState(succ_state)
  , ivId(-1)
{
  ivYaw = angle_state_2_cell(ivState.getYaw(), angle_bin_size);

  ivHashTag = calc_hash_tag(ivX, ivY, ivYaw, leg, max_hash_size);
}


PlanningState::PlanningState(const State& s, double cell_size, double angle_bin_size, int max_hash_size, const PlanningState *pred_state, const PlanningState *succ_state)
  : ivState(s)
  , ivX(state_2_cell(s.getX(), cell_size))
  , ivY(state_2_cell(s.getY(), cell_size))
  , ivYaw(angle_state_2_cell(s.getYaw(), angle_bin_size))
  , ivpPredState(pred_state)
  , ivpSuccState(succ_state)
  , ivId(-1)
  , ivHashTag(calc_hash_tag(ivX, ivY, ivYaw, s.getLeg(), max_hash_size))
{}


PlanningState::PlanningState(const PlanningState& s)
  : ivState(s.getState())
  , ivX(s.getX())
  , ivY(s.getY())
  , ivYaw(s.getYaw())
  , ivpPredState(s.getPredState())
  , ivpSuccState(s.getSuccState())
  , ivId(s.getId())
  , ivHashTag(s.getHashTag())
{}


PlanningState::~PlanningState()
{}


bool
PlanningState::operator ==(const PlanningState& s2)
const
{
  // First test the hash tag. If they differ, the states are definitely
  // different.
  if (ivHashTag != s2.getHashTag())
    return false;

  // other variables may be ignored, because they are
  // selected by x and y
  return (ivX == s2.getX() && ivY == s2.getY() &&
          ivYaw == s2.getYaw() && ivState.getLeg() == s2.ivState.getLeg());
}


bool
PlanningState::operator !=(const PlanningState& s2)
const
{
  return ivHashTag != s2.getHashTag();
}


const State &PlanningState::getState() const
{
  return ivState;
}
} // end of namespace
