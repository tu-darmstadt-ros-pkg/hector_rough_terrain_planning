//=================================================================================================
// Copyright (c) 2013, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef STEP_COST_ESTIMATOR_H
#define STEP_COST_ESTIMATOR_H

#include <footstep_planner/PlanningState.h>
#include <footstep_planner/State.h>
#include <footstep_planner/helper.h>

namespace footstep_planner
{
class StepCostEstimator
{
public:
  enum StepCostEstimatorType
  {
    CONST_STEP_COST_ESTIMATOR             = 0,
    EUCLIDEAN_STEP_COST_ESTIMATOR         = 1,
    GPR_STEP_COST_ESTIMATOR               = 2,
    MAP_STEP_COST_ESTIMATOR               = 3,
    BOUNDARY_STEP_COST_ESTIMATOR          = 4,
    DYNAMICS_STEP_COST_ESTIMATOR          = 5,
    GROUND_CONTACT_STEP_COST_ESTIMATOR    = 6
  };

  typedef boost::shared_ptr<StepCostEstimator> Ptr;
  typedef boost::shared_ptr<const StepCostEstimator> ConstPtr;

  StepCostEstimator(StepCostEstimatorType step_cost_estimator_type);
  StepCostEstimator(StepCostEstimator::Ptr step_cost_estimater, StepCostEstimatorType step_cost_estimator_type);
  virtual ~StepCostEstimator() {}

  StepCostEstimatorType getEstimatorType() { return step_cost_estimator_type; }

  virtual const char *getName() const = 0;

  virtual double getCost(const State &left_foot, const State &right_foot, const State &swing_foot, double &risk_cost);

protected:
  StepCostEstimator::Ptr step_cost_estimater;
  StepCostEstimatorType step_cost_estimator_type;
};
}

#endif
