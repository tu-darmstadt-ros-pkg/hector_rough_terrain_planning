#include <footstep_planner/step_cost_estimators/euclidean_step_cost_estimator.h>

namespace footstep_planner
{
EuclideanStepCostEstimator::EuclideanStepCostEstimator()
  : StepCostEstimator(EUCLIDEAN_STEP_COST_ESTIMATOR) {}

EuclideanStepCostEstimator::EuclideanStepCostEstimator(StepCostEstimator::Ptr step_cost_estimater)
  : StepCostEstimator(step_cost_estimater, EUCLIDEAN_STEP_COST_ESTIMATOR) {}

const char *EuclideanStepCostEstimator::getName() const
{
  return "euclidean step cost estimator";
}

double EuclideanStepCostEstimator::getCost(const State &left_foot, const State &right_foot, const State &swing_foot, double& risk_cost)
{
  const State& swing_foot_before = swing_foot.getLeg() == LEFT ? left_foot : right_foot;
  double dist = 0.5*euclidean_distance(swing_foot_before.getX(), swing_foot_before.getY(), swing_foot.getX(), swing_foot.getY());

  return dist + StepCostEstimator::getCost(left_foot, right_foot, swing_foot, risk_cost);
}
}
