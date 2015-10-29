#include <footstep_planner/step_cost_estimators/const_step_cost_estimator.h>

namespace footstep_planner
{
ConstStepCostEstimator::ConstStepCostEstimator(double const_step_cost)
  : StepCostEstimator(CONST_STEP_COST_ESTIMATOR)
  , const_step_cost(const_step_cost) {}

ConstStepCostEstimator::ConstStepCostEstimator(StepCostEstimator::Ptr step_cost_estimater, double const_step_cost)
  : StepCostEstimator(step_cost_estimater, CONST_STEP_COST_ESTIMATOR)
  , const_step_cost(const_step_cost) {}

const char *ConstStepCostEstimator::getName() const
{
  return "const step cost estimator";
}

double ConstStepCostEstimator::getCost(const State &left_foot, const State &right_foot, const State &swing_foot, double& risk_cost)
{
  return const_step_cost + StepCostEstimator::getCost(left_foot, right_foot, swing_foot, risk_cost);
}
}
