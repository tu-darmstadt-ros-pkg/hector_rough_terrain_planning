#include <footstep_planner/step_cost_estimators/gpr_step_cost_estimator.h>

namespace footstep_planner
{
GprStepCostEstimator::GprStepCostEstimator(const std::string &filename)
  : StepCostEstimator(GPR_STEP_COST_ESTIMATOR)
  , gpr(flor_gpr::FlorFootstepPlannerGPR::Ptr(new flor_gpr::FlorFootstepPlannerGPR(filename)))
{}

GprStepCostEstimator::GprStepCostEstimator(StepCostEstimator::Ptr step_cost_estimater, const std::string &filename)
  : StepCostEstimator(step_cost_estimater, GPR_STEP_COST_ESTIMATOR)
  , gpr(flor_gpr::FlorFootstepPlannerGPR::Ptr(new flor_gpr::FlorFootstepPlannerGPR(filename)))
{}

const char *GprStepCostEstimator::getName() const
{
  return "gpr step cost estimator";
}

double GprStepCostEstimator::getCost(const State &left_foot, const State &right_foot, const State &swing_foot, double &risk_cost)
{
  if (!gpr)
  {
    ROS_ERROR_THROTTLE(10, "Can't compute GPR step cost: GPR not initialized!");
    return 0.0;
  }

  // generate data point for evaluation with gpr
  flor_gpr::footstep_planner_gpr_data_point data_point;

/// TODO: fix it
//  left_foot.getStep(data_point.left_foot);
//  right_foot.getStep(data_point.right_foot);
//  swing_foot.getStep(data_point.swing_foot);
  data_point.terrain_type = flor_gpr::footstep_planner_gpr_data_point::flat;

  double gpr_step_cost;
  gpr->EvaluateCost(data_point, gpr_step_cost);

  if (gpr_step_cost < 0.0)
    gpr_step_cost = 0.0;

  // check for NAN, this may happen, when doing reinforcement learning from scratch
  if (isnan(gpr_step_cost))
    gpr_step_cost = 0.1;

  //ROS_INFO("Step Cost %f",step_cost);

  //ROS_INFO("------- foot: %u --- index: %u -----------------------", data_point.swing_foot.foot_index, data_point.swing_foot.step_index);
  //ROS_INFO("%f %f %f / %f %f %f", vec[0], vec[1], vec[2], vec[3], vec[4], vec[5]);
  //ROS_INFO("%f %f %f / %f %f %f", vec[6], vec[7], vec[8], vec[9], vec[10], vec[11]);
  //ROS_INFO("%f %f / %f", cell_2_state(tuple.get<0>(), ivCellSize), cell_2_state(tuple.get<1>(), ivCellSize), angle_cell_2_state(tuple.get<2>(), ivNumAngleBins));
  //ROS_INFO("%f %f / %f", cell_2_state(tuple.get<3>(), ivCellSize), cell_2_state(tuple.get<4>(), ivCellSize), angle_cell_2_state(tuple.get<5>(), ivNumAngleBins));

  risk_cost += gpr_step_cost;
  return gpr_step_cost + StepCostEstimator::getCost(left_foot, right_foot, swing_foot, risk_cost);
}
}
