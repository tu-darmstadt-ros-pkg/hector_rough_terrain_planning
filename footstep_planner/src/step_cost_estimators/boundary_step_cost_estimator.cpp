#include <footstep_planner/step_cost_estimators/boundary_step_cost_estimator.h>

namespace footstep_planner
{
BoundaryStepCostEstimator::BoundaryStepCostEstimator(double max_diff_z, double long_step_dist, double min_yaw_seperation_enlargement, double yaw_enlarged_min_seperation, double cost_roll_abs, double cost_pitch_abs, double cost_yaw_rel, double cost_height_diff_rel)
  : StepCostEstimator(BOUNDARY_STEP_COST_ESTIMATOR)
  , max_diff_z(max_diff_z)
  , long_step_dist(long_step_dist)
  , min_yaw_seperation_enlargement(min_yaw_seperation_enlargement)
  , yaw_enlarged_min_seperation(yaw_enlarged_min_seperation)
  , cost_roll_abs(cost_roll_abs)
  , cost_pitch_abs(cost_pitch_abs)
  , cost_yaw_rel(cost_yaw_rel)
  , cost_height_diff_rel(cost_height_diff_rel)
{}

BoundaryStepCostEstimator::BoundaryStepCostEstimator(StepCostEstimator::Ptr step_cost_estimater, double max_diff_z, double long_step_dist, double min_yaw_seperation_enlargement, double yaw_enlarged_min_seperation, double cost_roll_abs, double cost_pitch_abs, double cost_yaw_rel, double cost_height_diff_rel)
  : StepCostEstimator(step_cost_estimater, BOUNDARY_STEP_COST_ESTIMATOR)
  , max_diff_z(max_diff_z)
  , long_step_dist(long_step_dist)
  , min_yaw_seperation_enlargement(min_yaw_seperation_enlargement)
  , yaw_enlarged_min_seperation(yaw_enlarged_min_seperation)
  , cost_roll_abs(cost_roll_abs)
  , cost_pitch_abs(cost_pitch_abs)
  , cost_yaw_rel(cost_yaw_rel)
  , cost_height_diff_rel(cost_height_diff_rel)
{}

const char *BoundaryStepCostEstimator::getName() const
{
  return "boundary step cost estimator";
}

double BoundaryStepCostEstimator::getCost(const State &left_foot, const State &right_foot, const State &swing_foot, double& risk_cost)
{
  const State& stand_foot = swing_foot.getLeg() == LEFT ? right_foot : left_foot;
  const State& swing_foot_before = swing_foot.getLeg() == LEFT ? left_foot : right_foot;

  double diff_z = std::abs(swing_foot.getZ()-stand_foot.getZ());

  // add costs
  double cost = 0.0;
  double risk = 0.0;

  if (diff_z >= max_diff_z)
    risk = 100.0;
  else
  {
    // determine additional costs
    tf::Transform step = stand_foot.getPose().inverse() * swing_foot.getPose();

    // all long steps should be more expensive
    if (step.getOrigin().x() > long_step_dist)
      cost += step.getOrigin().x()-long_step_dist;

    // get yaw diffs
    double stance_yaw_diff = angles::shortest_angular_distance(swing_foot_before.getYaw(), stand_foot.getYaw());
    double swing_foot_yaw_diff = angles::shortest_angular_distance(stand_foot.getYaw(), swing_foot.getYaw());

    // if foot is turned step more outside
    if (std::abs(swing_foot_yaw_diff) >= min_yaw_seperation_enlargement && std::abs(step.getOrigin().y()) <= yaw_enlarged_min_seperation)
      risk = 100.0;

    // determine costs changing yaw: increasing turn rate may be expensiv but decreasing is free
    double turn_rate_diff = swing_foot_yaw_diff - stance_yaw_diff;

    if (turn_rate_diff * swing_foot_yaw_diff <= 0.0) // check for different sign
      turn_rate_diff = 0.0; // decreasing turn rate towards zero is always free

    // determine risk
    risk += cost_roll_abs * std::abs(swing_foot.getRoll());
    risk += cost_pitch_abs * std::abs(swing_foot.getPitch());
    risk += cost_yaw_rel * std::abs(turn_rate_diff);
    risk += cost_height_diff_rel * diff_z;
  }

  risk_cost += risk;
  return risk*risk + cost + StepCostEstimator::getCost(left_foot, right_foot, swing_foot, risk_cost);
}
}
