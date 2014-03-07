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

#ifndef MAP_STEP_COST_ESTIMATOR_H
#define MAP_STEP_COST_ESTIMATOR_H

#include <fstream>

#include <footstep_planner/step_cost_estimators/step_cost_estimator.h>
#include <footstep_planner/step_cost_estimators/step_cost_key.h>

namespace footstep_planner
{
class MapStepCostEstimator
  : public StepCostEstimator
{
public:
  MapStepCostEstimator(const std::string &filename);
  MapStepCostEstimator(StepCostEstimator::Ptr step_cost_estimater, const std::string &filename);

  const char *getName() const;

  double getCost(const State &left_foot, const State &right_foot, const State &swing_foot, double &risk_cost);

protected:
  void insert(const std::vector<double> &key, const std::pair<double, double> &entry);

  void loadFromFile(const string &filename);

  template <class T> void read(std::ifstream &in, T &value)
  {
    in.read((char*) &value, sizeof(value));
  }

  // boundaries
  float min_x, max_x;
  float min_y, max_y;
  float min_yaw, max_yaw;

  // resolution
  double cell_size;
  uint32_t num_angle_bin;
  double angle_bin_size;

  // map
  boost::unordered_map<StepCostKey, std::pair<double, double> > cost_map;
};
}

#endif
