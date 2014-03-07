#include <footstep_planner/step_cost_estimators/dynamics_step_cost_estimator.h>

namespace footstep_planner
{
DynamicsStepCostEstimator::DynamicsStepCostEstimator(const FootstepPlannerEnvironment& planner_environment, double lower_step_limit, double upper_step_limit, double max_near_distance)
  : StepCostEstimator(DYNAMICS_STEP_COST_ESTIMATOR)
  , planner_environment(planner_environment)
  , lower_step_limit(lower_step_limit)
  , upper_step_limit(upper_step_limit)
  , max_near_distance_sq(max_near_distance*max_near_distance)
{}

DynamicsStepCostEstimator::DynamicsStepCostEstimator(StepCostEstimator::Ptr step_cost_estimater, const FootstepPlannerEnvironment& planner_environment, double lower_step_limit, double upper_step_limit, double max_near_distance)
  : StepCostEstimator(step_cost_estimater, DYNAMICS_STEP_COST_ESTIMATOR)
  , planner_environment(planner_environment)
  , lower_step_limit(lower_step_limit)
  , upper_step_limit(upper_step_limit)
  , max_near_distance_sq(max_near_distance*max_near_distance)
{}

const char *DynamicsStepCostEstimator::getName() const
{
  return "dynamics step cost estimator";
}

double DynamicsStepCostEstimator::getCost(const State &left_foot, const State &right_foot, const State &swing_foot, double& risk_cost)
{
  double risk = 0.0;

  // only in specific modes we must care about robot dynamics
  if (planner_environment.getPlanningMode() == flor_footstep_planner_msgs::FootstepPlan::MODE_WALK)
  {
    // check distance to start pose
    State robot_start;
    planner_environment.getStartState(robot_start);
    double dist_sq = euclidean_distance_sq(robot_start.getX(), robot_start.getY(), swing_foot.getX(), swing_foot.getY());
    double factor = std::max(0.0, (max_near_distance_sq-dist_sq)/max_near_distance_sq);

    // check distance to goal pose
    if (factor < 1.0)
    {
      State robot_goal;
      planner_environment.getGoalState(robot_goal);
      dist_sq = euclidean_distance_sq(robot_goal.getX(), robot_goal.getY(), swing_foot.getX(), swing_foot.getY());
      factor = std::max(factor, (max_near_distance_sq-dist_sq)/max_near_distance_sq);
    }

    factor = 1.0-factor;
    double max_step_dist = lower_step_limit + upper_step_limit*factor*factor;

    // determine step distance
    const State& swing_foot_before = swing_foot.getLeg() == LEFT ? left_foot : right_foot;
    double step_dist_sq = euclidean_distance_sq(swing_foot_before.getX(), swing_foot_before.getY(), swing_foot.getX(), swing_foot.getY());

    if (step_dist_sq > max_step_dist*max_step_dist)
      risk = 100.0;

    //ROS_INFO("%f %f | %f %f | %f | %f", min_step_dist, max_step_dist, std::sqrt(step_dist_sq), std::sqrt(dist_sq), factor, risk);
  }

  risk_cost += risk;
  return StepCostEstimator::getCost(left_foot, right_foot, swing_foot, risk_cost);
}
}
