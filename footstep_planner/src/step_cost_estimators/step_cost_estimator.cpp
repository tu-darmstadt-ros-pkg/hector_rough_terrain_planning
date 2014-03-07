#include <footstep_planner/step_cost_estimators/step_cost_estimator.h>

namespace footstep_planner
{
StepCostEstimator::StepCostEstimator(StepCostEstimatorType step_cost_estimator_type)
  : step_cost_estimator_type(step_cost_estimator_type) {}

StepCostEstimator::StepCostEstimator(StepCostEstimator::Ptr step_cost_estimater, StepCostEstimatorType step_cost_estimator_type)
  : step_cost_estimater(step_cost_estimater)
  , step_cost_estimator_type(step_cost_estimator_type) {}

double StepCostEstimator::getCost(const State &left_foot, const State &right_foot, const State &swing_foot, double &risk_cost)
{
//  if (left_foot.getLeg() != LEFT)
//    ROS_WARN("StepCostEstimator: Given left_foot seems is not defined as left foot!");
//  if (right_foot.getLeg() != RIGHT)
//    ROS_WARN("StepCostEstimator: Given right_foot seems is not defined as right foot!");

  if (step_cost_estimater)
    return step_cost_estimater->getCost(left_foot, right_foot, swing_foot, risk_cost);

  return 0.0;
}
}
