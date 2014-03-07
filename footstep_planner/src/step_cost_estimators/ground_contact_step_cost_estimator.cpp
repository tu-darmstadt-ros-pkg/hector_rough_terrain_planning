#include <footstep_planner/step_cost_estimators/ground_contact_step_cost_estimator.h>

namespace footstep_planner
{
GroundContactStepCostEstimator::GroundContactStepCostEstimator(StepCostEstimator::Ptr step_cost_estimater, double min_contact_support)
  : StepCostEstimator(step_cost_estimater, step_cost_estimater->getEstimatorType())
  , min_contact_support(min_contact_support)
{}

const char *GroundContactStepCostEstimator::getName() const
{
  return step_cost_estimater->getName();
}

double GroundContactStepCostEstimator::getCost(const State &left_foot, const State &right_foot, const State &swing_foot, double& risk_cost)
{
  double scaling = 1.0;
  double risk = 0.0;

  if (swing_foot.getGroundContactSupport() < 1.0)
  {
    if (swing_foot.getGroundContactSupport() > min_contact_support)
      scaling = 1.0/swing_foot.getGroundContactSupport();
    else
    {
      scaling = 1.0/min_contact_support;
      risk = 100.0;
    }
  }

  risk_cost += risk;
  return StepCostEstimator::getCost(left_foot, right_foot, swing_foot, risk_cost) * scaling;
}
}
