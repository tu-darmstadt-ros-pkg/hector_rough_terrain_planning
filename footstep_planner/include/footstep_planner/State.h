// SVN $HeadURL: https://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_stacks/humanoid_navigation/footstep_planner/include/footstep_planner/State2.h $
// SVN $Id: State2.h 2793 2012-06-07 10:37:53Z Garimort.Johannes $

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

#ifndef FOOTSTEP_PLANNER_STATE_H_
#define FOOTSTEP_PLANNER_STATE_H_

#include <footstep_planner/helper.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <flor_atlas_msgs/AtlasBehaviorFootData.h>
#include <flor_footstep_planner_msgs/flor_footstep_planner_msgs.h>
#include <flor_footstep_planner_msgs/StepTarget.h>


namespace footstep_planner
{
/**
 * @brief A class representing the robot's pose (i.e. position and
 * orientation) in the (continuous) world view. More precisely a state
 * points to the robot's supporting leg.
 */
class State
{
public:
  State();
  State(double x, double y, double z, double roll, double pitch, double yaw, double swing_height, double lift_height, double step_duration, double sway_duration, Leg leg);
  State(const geometry_msgs::Vector3 &position, double roll, double pitch, double yaw, double swing_height, double lift_height, double step_duration, double sway_duration, Leg leg);
  State(const geometry_msgs::Vector3 &position, const geometry_msgs::Vector3 &normal, double yaw, double swing_height, double lift_height, double step_duration, double sway_duration, Leg leg);
  State(const geometry_msgs::Pose &pose, double swing_height, double lift_height, double step_duration, double sway_duration, Leg leg);
  State(const tf::Transform &t, double swing_height, double lift_height, double step_duration, double sway_duration, Leg leg);
  ~State();

  /**
   * @brief Compare two states on equality of x, y, theta, leg upon
   * a certain degree of float precision.
   */
  bool operator ==(const State& s2) const;

  /**
   * @brief Inequality operator for two states (negates the equality
   * operator).
   */
  bool operator !=(const State& s2) const;

  void setX(double x) { ivPose.getOrigin().setX(x); }
  void setY(double y) { ivPose.getOrigin().setY(y); }
  void setZ(double z) { ivPose.getOrigin().setZ(z); }
  void setRoll(double roll) { ivRoll = roll; ivPose.getBasis().setRPY(ivRoll, ivPitch, ivYaw); recomputeNormal(); }
  void setPitch(double pitch) { ivPitch = pitch; ivPose.getBasis().setRPY(ivRoll, ivPitch, ivYaw); recomputeNormal(); }
  void setYaw(double yaw);
  void setRPY(double roll, double pitch, double yaw);
  void setNormal(const geometry_msgs::Vector3 &normal);
  void setNormal(double x, double y, double z);
  void setSwingHeight(double swing_height) { ivSwingHeight = swing_height; }
  void setLiftHeight(double lift_height) { ivLiftHeight = lift_height; }
  void setStepDuration(double step_duration) { ivStepDuration = step_duration; }
  void setSwayDuration(double sway_duration) { ivSwayDuration = sway_duration; }
  void setLeg(Leg leg) { ivLeg = leg; }
  void setKneeNominal(double knee_nominal) { ivKneeNominal = knee_nominal; }
  void setGroundContactSupport(double ground_contact_support) { ivGroundContactSupport = ground_contact_support; }

  double getX() const { return ivPose.getOrigin().getX(); }
  double getY() const { return ivPose.getOrigin().getY(); }
  double getZ() const { return ivPose.getOrigin().getZ(); }
  double getRoll() const { return ivRoll; }
  double getPitch() const { return ivPitch; }
  double getYaw() const { return ivYaw; }
  const geometry_msgs::Vector3& getNormal() const { return ivNormal; }
  double getNormalX() const { return ivNormal.x; }
  double getNormalY() const { return ivNormal.y; }
  double getNormalZ() const { return ivNormal.z; }
  double getSwingHeight() const { return ivSwingHeight; }
  double getLiftHeight() const { return ivLiftHeight; }
  double getStepDuration() const { return ivStepDuration; }
  double getSwayDuration() const { return ivSwayDuration; }
  Leg getLeg() const { return ivLeg; }
  double getKneeNominal() const { return ivKneeNominal; }
  double getGroundContactSupport() const { return ivGroundContactSupport; }

  const tf::Pose &getPose() const { return ivPose; }
  tf::Pose &getPose() { return ivPose; }
  void getStep(flor_footstep_planner_msgs::StepTarget &step) const;
  void getFootData(flor_atlas_msgs::AtlasBehaviorFootData& foot_data) const;

private:
  void recomputeNormal();

  tf::Pose ivPose;

  /// The robot's orientation.
  double ivRoll;
  double ivPitch;
  double ivYaw;
  /// The normal of foot in world.
  geometry_msgs::Vector3 ivNormal;
  /// swing and lift height
  double ivSwingHeight;
  double ivLiftHeight;
  /// step and sway duration
  double ivStepDuration;
  double ivSwayDuration;
  /// The robot's supporting leg.
  Leg ivLeg;

  /// leg's knee nominal
  double ivKneeNominal;

  /// percentage of ground contact support (0.0 - 1.0 = 100%)
  double ivGroundContactSupport;
};
}
#endif /* FOOTSTEP_PLANNER_STATE_H_ */
