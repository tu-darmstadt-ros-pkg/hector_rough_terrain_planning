// SVN $HeadURL: https://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_stacks/humanoid_navigation/footstep_planner/src/State2.cpp $
// SVN $Id: State2.cpp 2852 2012-06-26 14:07:35Z Garimort.Johannes $

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

#include <footstep_planner/State.h>


namespace footstep_planner
{
State::State()
  : ivRoll(0.0)
  , ivPitch(0.0)
  , ivYaw(0.0)
  , ivSwingHeight(0.0)
  , ivLiftHeight(0.0)
  , ivStepDuration(0.0)
  , ivSwayDuration(0.0)
  , ivLeg(NOLEG)
  , ivKneeNominal(0.0)
  , ivGroundContactSupport(0.0)
{
  ivNormal.x = 0.0;
  ivNormal.y = 0.0;
  ivNormal.z = 1.0;
}

State::State(double x, double y, double z, double roll, double pitch, double yaw, double swing_height, double lift_height, double step_duration, double sway_duration, Leg leg)
  : ivRoll(roll)
  , ivPitch(pitch)
  , ivYaw(yaw)
  , ivSwingHeight(swing_height)
  , ivLiftHeight(lift_height)
  , ivStepDuration(step_duration)
  , ivSwayDuration(sway_duration)
  , ivLeg(leg)
  , ivKneeNominal(0.0)
  , ivGroundContactSupport(1.0)
{
  ivPose.setOrigin(tf::Vector3(x, y, z));
  ivPose.getBasis().setRPY(ivRoll, ivPitch, ivYaw);

  recomputeNormal();
}

State::State(const geometry_msgs::Vector3 &position, double roll, double pitch, double yaw, double swing_height, double lift_height, double step_duration, double sway_duration, Leg leg)
  : ivRoll(roll)
  , ivPitch(pitch)
  , ivYaw(yaw)
  , ivSwingHeight(swing_height)
  , ivLiftHeight(lift_height)
  , ivStepDuration(step_duration)
  , ivSwayDuration(sway_duration)
  , ivLeg(leg)
  , ivKneeNominal(0.0)
  , ivGroundContactSupport(1.0)
{
  ivPose.setOrigin(tf::Vector3(position.x, position.y, position.z));
  ivPose.getBasis().setRPY(ivRoll, ivPitch, ivYaw);

  recomputeNormal();
}

State::State(const geometry_msgs::Vector3 &position, const geometry_msgs::Vector3 &normal, double yaw, double swing_height, double lift_height, double step_duration, double sway_duration, Leg leg)
  : ivYaw(yaw)
  , ivSwingHeight(swing_height)
  , ivLiftHeight(lift_height)
  , ivStepDuration(step_duration)
  , ivSwayDuration(sway_duration)
  , ivLeg(leg)
  , ivKneeNominal(0.0)
  , ivGroundContactSupport(1.0)
{
  ivPose.setOrigin(tf::Vector3(position.x, position.y, position.z));
  ivPose.getBasis().setRPY(ivRoll, ivPitch, ivYaw);

  setNormal(normal);
}

State::State(const geometry_msgs::Pose &pose, double swing_height, double lift_height, double step_duration, double sway_duration, Leg leg)
  : ivSwingHeight(swing_height)
  , ivLiftHeight(lift_height)
  , ivStepDuration(step_duration)
  , ivSwayDuration(sway_duration)
  , ivLeg(leg)
  , ivKneeNominal(0.0)
  , ivGroundContactSupport(1.0)
{
  tf::poseMsgToTF(pose, ivPose);
  ivPose.getBasis().getRPY(ivRoll, ivPitch, ivYaw);

  recomputeNormal();
}

State::State(const tf::Transform &t, double swing_height, double lift_height, double step_duration, double sway_duration, Leg leg)
  : ivSwingHeight(swing_height)
  , ivLiftHeight(lift_height)
  , ivStepDuration(step_duration)
  , ivSwayDuration(sway_duration)
  , ivLeg(leg)
  , ivKneeNominal(0.0)
  , ivGroundContactSupport(1.0)
{
  ivPose = t;
  ivPose.getBasis().getRPY(ivRoll, ivPitch, ivYaw);

  recomputeNormal();
}

State::~State()
{}

bool State::operator ==(const State& s2) const
{
  return (fabs(getX() - s2.getX()) < FLOAT_CMP_THR &&
          fabs(getY() - s2.getY()) < FLOAT_CMP_THR &&
          fabs(getZ() - s2.getZ()) < FLOAT_CMP_THR &&
          fabs(angles::shortest_angular_distance(ivRoll, s2.ivRoll)) < FLOAT_CMP_THR &&
          fabs(angles::shortest_angular_distance(ivPitch, s2.ivPitch)) < FLOAT_CMP_THR &&
          fabs(angles::shortest_angular_distance(ivYaw, s2.ivYaw)) < FLOAT_CMP_THR &&
          ivLeg == s2.getLeg());
}

bool State::operator !=(const State& s2) const
{
  return not (*this == s2);
}

void State::setYaw(double yaw)
{
  ivYaw = yaw;
  ivPose.getBasis().setRPY(ivRoll, ivPitch, ivYaw);
  recomputeNormal();
}

void State::setRPY(double roll, double pitch, double yaw)
{
  ivRoll = roll;
  ivPitch = pitch;
  ivYaw = yaw;

  ivPose.getBasis().setRPY(ivRoll, ivPitch, ivYaw);
  recomputeNormal();
}

void State::setNormal(const geometry_msgs::Vector3 &normal)
{
  ivNormal = normal;

  if (ivNormal.z > 0.99)
  {
    ivNormal.x = 0.0;
    ivNormal.y = 0.0;
    ivNormal.z = 1.0;
    ivRoll = 0.0;
    ivPitch = 0.0;
  }
  else
  {
    // get roll and pitch
    flor_footstep_planner_msgs::normalToRP(ivYaw, ivNormal, ivRoll, ivPitch);
  }

  ivPose.getBasis().setRPY(ivRoll, ivPitch, ivYaw);
}

void State::setNormal(double x, double y, double z)
{
  geometry_msgs::Vector3 n;
  n.x = x;
  n.y = y;
  n.z = z;
  setNormal(n);
}

void State::getStep(flor_footstep_planner_msgs::StepTarget &step) const
{
  step.step_index = 0;
  step.foot_index = (ivLeg == footstep_planner::LEFT) ? flor_atlas_msgs::AtlasBehaviorFootData::FOOT_LEFT : flor_atlas_msgs::AtlasBehaviorFootData::FOOT_RIGHT;
  getFootData(step.foot);
  step.step_duration = ivStepDuration;
  step.sway_duration = ivSwayDuration;
  step.swing_height = ivSwingHeight;
  step.lift_height = ivLiftHeight;
  step.knee_nominal = ivKneeNominal;
}

void State::getFootData(flor_atlas_msgs::AtlasBehaviorFootData& foot_data) const
{
  foot_data.position.x = getX();
  foot_data.position.y = getY();
  foot_data.position.z = getZ();
  foot_data.yaw = ivYaw;
  foot_data.normal = ivNormal;
}

void State::recomputeNormal()
{
  if (ivRoll < 0.01 && ivPitch < 0.01)
  {
    ivNormal.x = 0.0;
    ivNormal.y = 0.0;
    ivNormal.z = 1.0;
  }
  else
  {
    // get normal
    flor_footstep_planner_msgs::RPYToNormal(ivRoll, ivPitch, ivYaw, ivNormal);
  }
}

} // end of namespace
