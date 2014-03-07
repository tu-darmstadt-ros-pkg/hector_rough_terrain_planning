// SVN $HeadURL: http://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_stacks/humanoid_navigation/footstep_planner/include/footstep_planner/FootstepPlannerEnvironment.h $
// SVN $Id: FootstepPlannerEnvironment.h 4148 2013-05-13 14:55:16Z garimort@informatik.uni-freiburg.de $

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

#ifndef FOOTSTEP_PLANNER_FOOTSTEPPLANNERENVIRONMENT_H_
#define FOOTSTEP_PLANNER_FOOTSTEPPLANNERENVIRONMENT_H_

#include <math.h>

#include <vector>
#include <tr1/unordered_set>
#include <tr1/hashtable.h>

#include <sbpl/headers.h>

#include <nav_msgs/OccupancyGrid.h>
#include <humanoid_nav_msgs/ClipFootstep.h>

#include <flor_footstep_planner_msgs/FootstepPlannerParams.h>
#include <flor_gpr/flor_footstep_planner_gpr.h>

#include <footstep_planner/helper.h>
#include <footstep_planner/Footstep.h>
#include <footstep_planner/PlanningState.h>
#include <footstep_planner/State.h>
#include <footstep_planner/terrain_model.h>
#include <footstep_planner/heuristics/Heuristic.h>
#include <footstep_planner/heuristics/PathCostHeuristic.h>
#include <footstep_planner/step_cost_estimators/step_cost_estimator.h>
#include <footstep_planner/step_cost_estimators/boundary_step_cost_estimator.h>
#include <footstep_planner/step_cost_estimators/const_step_cost_estimator.h>
#include <footstep_planner/step_cost_estimators/dynamics_step_cost_estimator.h>
#include <footstep_planner/step_cost_estimators/euclidean_step_cost_estimator.h>
#include <footstep_planner/step_cost_estimators/ground_contact_step_cost_estimator.h>
#include <footstep_planner/step_cost_estimators/gpr_step_cost_estimator.h>
#include <footstep_planner/step_cost_estimators/map_step_cost_estimator.h>


#define EUCLIDEAN_STEP_COST   flor_footstep_planner_msgs::FootstepPlannerParams::EUCLIDEAN_STEP_COST_ESTIMATOR
#define GPR_STEP_COST         flor_footstep_planner_msgs::FootstepPlannerParams::GPR_STEP_COST_ESTIMATOR
#define MAP_STEP_COST         flor_footstep_planner_msgs::FootstepPlannerParams::MAP_STEP_COST_ESTIMATOR
#define BOUNDARY_STEP         flor_footstep_planner_msgs::FootstepPlannerParams::BOUNDARY_STEP_COST_ESTIMATOR
#define DYNAMICS_STEP         flor_footstep_planner_msgs::FootstepPlannerParams::DYNAMICS_STEP_COST_ESTIMATOR

namespace footstep_planner
{
struct environment_params
{
  std::vector<Footstep> footstep_set;

  /// Defines the area of performable (discrete) steps.
  std::vector<std::pair<int, int> > step_range;

  // foot paramaters
  geometry_msgs::Vector3 foot_size;
  geometry_msgs::Vector3 foot_origin_shift;
  double foot_seperation;

  // upper body parameters
  geometry_msgs::Vector3 upper_body_size;
  geometry_msgs::Vector3 upper_body_origin_shift;

  double max_step_range_x, max_step_range_y, max_step_range_theta;
  double max_inverse_step_range_x, max_inverse_step_range_y, max_inverse_step_range_theta;
  double max_step_range_width;        // maximal step distance based on step polygon
  double max_step_dist;               // maximal distance in footstep primitives
  double swing_height, lift_height;   // swing height and step duration used as default
  double step_duration, sway_duration;

  // parameter for const step cost estimator
  double const_step_cost;
  // parameters for boundary step cost estimator
  double boundary_est_max_diff_z;
  double boundary_est_long_step_dist;
  double boundary_est_min_yaw_seperation_enlargement;
  double boundary_est_yaw_enlarged_min_seperation;
  double boundary_est_cost_roll_abs;
  double boundary_est_cost_pitch_abs;
  double boundary_est_cost_yaw_rel;
  double boundary_est_cost_height_diff_rel;
  // parameters for dynamic step cost estimator
  double dynamic_est_lower_step_limit;
  double dynamic_est_upper_step_limit;
  double dynamic_est_max_near_distance;

  // parameters for post processing step
  double post_processing_sw_max_swing_dist;    // max swing distance (in direction the stand foot faces) before sway duration will be adjusted
  double post_processing_sw_min_normal_z;      // min value in z axis of normal before sway duration will be adjusted
  double post_processing_sw_adjusted;          // adjusted sway duration
  double post_processing_toe_max_turn_rate;    // max turn rate [rad] before TOE-OFF will be disabled
  double post_processing_kn_max_step_down;     // max step down distance (positive) [m] before knee nominal will be adjusted
  double post_processing_kn_adjusted;          // adjusted knee nominal

  double diff_angle_cost;

  int    collision_check_type;        // Type of collision check
  int    collision_check_accuracy;    // Whether to check just the foot's inner circle (0), the hole
                                      // outer circle (1) or exactly the foot's bounding box (2) for collision.

  // parameters for foot contact support check
  unsigned int foot_contact_min_sampling_steps_x;   // min number of sampling steps in x
  unsigned int foot_contact_min_sampling_steps_y;   // min number of sampling steps in y
  unsigned int foot_contact_max_sampling_steps_x;   // max number of sampling steps in x
  unsigned int foot_contact_max_sampling_steps_y;   // max number of sampling steps in y
  double foot_contact_max_intrusion_z;          // how deep the foot may intrude into other objects (z axis only)#
  double foot_contact_max_ground_clearance;     // maximal distance before a point is treated as "in the air"
  double foot_contact_minimal_support;          // minimal percentage of foot sole area must touch ground

  int    hash_table_size;             // Size of the hash table storing the planning states expanded during the search. (Also referred to by max_hash_size.)
  double cell_size;                   // The size of each grid cell used to discretize the robot positions.
  int    num_angle_bins;              // The number of bins used to discretize the robot orientations.
  double angle_bin_size;
  bool   forward_search;              // Whether to use forward search (1) or backward search (0).
  int    num_random_nodes;            // number of random neighbors for R*
  double random_node_distance;
  std::string step_cost_estimator_type_name;
  std::string heuristic_type_name;
  double heuristic_scale;             // Scaling factor of heuristic, in case it underestimates by a constant factor.
  bool use_terrain_model;
  std::string gpr_filename;
  std::string map_step_cost_filename;
};


/**
 * @brief A class defining a footstep planner environment for humanoid
 * robots used by the SBPL to perform planning tasks.
 *
 * The environment keeps track of all the planning states expanded during
 * the search. Each planning state can be accessed via its ID. Furthermore
 */
class FootstepPlannerEnvironment : public DiscreteSpaceInformation
{
public:
  // specialization of hash<int,int>, similar to standard boost::hash on pairs?
  struct IntPairHash{
  public:
    size_t operator()(std::pair<int, int> x) const throw() {
      size_t seed = std::tr1::hash<int>()(x.first);
      return std::tr1::hash<int>()(x.second) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    }
  };

  struct StepCostPair {
    StepCostPair(const PlanningState* state, const int cost)
      : state(state)
      , cost(cost)
    {}

    bool operator < (const StepCostPair& other)
    {
      return cost < other.cost;
    }

    const PlanningState* state;
    const int cost;
  };

  typedef std::vector<int> exp_states_t;
  typedef exp_states_t::const_iterator exp_states_iter_t;
  typedef std::tr1::unordered_set<std::pair<int,int>, IntPairHash > exp_states_2d_t;
  typedef exp_states_2d_t::const_iterator exp_states_2d_iter_t;

  enum collisionCheckType {CHECK_FOOT=1, CHECK_UPPER_BODY=2, FOOT_SUPPORT_AREA=4};

  /**
   * @param footstep_set The set of footsteps used for the path planning.
   * @param step_cost_estimator The estimator for cost of each step.
   * @param heuristic The heuristic used by the planner.
   * @param footsize_x Size of the foot in x direction.
   * @param footsize_y Size of the foot in y direction.
   * @param origin_foot_shift_x Shift in x direction from the foot's
   * center.
   * @param origin_foot_shift_y Shift in y direction from the foot's
   * center.
   * @param max_step_range_x The maximal translation in x direction
   * performable by the robot based on the step range polygon.
   * @param max_step_range_y The maximal translation in y direction
   * performable by the robot based on the step range polygon.
   * @param max_step_range_theta The maximal rotation performable by the
   * robot.
   * @param max_inverse_step_range_x The minimal translation in x direction
   * performable by the robot based on the step range polygon.
   * @param max_inverse_step_range_y The minimal translation in y direction
   * performable by the robot based on the step range polygon.
   * @param max_inverse_step_range_theta The minimal rotation performable by
   * the robot.
   * @param max_step_range_width The maximal step distance based on
   * the step range polygon.
   * @param max_step_dist The global maximal step distance (based on footstep
   * set and step range polygon)
   * @param default_step_cost The default costs for each step.
   * @param collision_check_accuracy Whether to check just the foot's
   * circumcircle (0), the incircle (1) or recursively the circumcircle
   * and the incircle for the whole foot (2) for collision.
   * @param hash_table_size Size of the hash table storing the planning
   * states expanded during the search.
   * @param cell_size The size of each grid cell used to discretize the
   * robot positions.
   * @param num_angle_bins The number of bins used to discretize the
   * robot orientations.
   * @param forward_search Whether to use forward search (1) or backward
   * search (0).
   */
  FootstepPlannerEnvironment(const environment_params& params);

  virtual ~FootstepPlannerEnvironment();

  /// @brief Get step cost estimator
  StepCostEstimator::Ptr initStepCostEstimator();
  StepCostEstimator::Ptr getStepCostEstimator() { return ivStepCostEstimatoPtr; }

  /// @brief Get heuristic
  boost::shared_ptr<Heuristic> initHeuristic();

  /**
   * @brief Update the robot's feet poses in the goal state.
   * @return The new IDs (left, right) of the planning state representing the
   * feet.
   */
  std::pair<int, int> updateGoal(const State& foot_left, const State& foot_right);

  /**
   * @brief Update the robot's feet poses in the start state.
   * @return The new IDs (left, right) of the planning states representing the
   * feet.
   */
  std::pair<int, int> updateStart(const State& foot_left, const State& right_right);

  /**
   * @brief Chooses best combination of start end goal foot
   */
  void setPlannerStartAndGoal();

  void updateGroundLevelMap(gridmap_2d::GridMap2DPtr map);
  void updateBodyLevelMap(gridmap_2d::GridMap2DPtr map);

  void updateTerrainModel(const flor_terrain_classifier::TerrainModel::ConstPtr &terrain_model);
  const TerrainModel::ConstPtr &getTerrainModel() const { return ivTerrainModel; }

  void setPlanningMode(uint8_t planning_mode) { ivPlannningMode = planning_mode; }
  int getPlanningMode() const { return ivPlannningMode; }

  /**
   * @return True iff the approximated upper body hits an obstacle.
   */
  bool occupiedUpperBody(const State& left, const State& right);

  /**
   * @return True iff the foot in State s is colliding with an
   * obstacle.
   */
  bool occupiedFoot(const State& s);

  /**
   * @brief Try to receive a state with a certain ID.
   *
   * @return True iff there is a state with such an ID.
   */
  bool getState(unsigned int id, State &s) const;

  void getStartState(State &left, State &right) const;
  void getStartState(State &robot) const;
  void getGoalState(State &left, State &right) const;
  void getGoalState(State &robot) const;

  /**
   * @brief Resets the current planning task (i.e. the start and goal
   * poses).
   */
  void reset();

  /// @return The number of expanded states during the search.
  int getNumExpandedStates() { return ivNumExpandedStates; }

  exp_states_2d_iter_t getExpandedStatesStart()
  {
    return ivExpandedStates.begin();
  }

  exp_states_2d_iter_t getExpandedStatesEnd()
  {
    return ivExpandedStates.end();
  }

  exp_states_iter_t getRandomStatesStart()
  {
    return ivRandomStates.begin();
  }

  exp_states_iter_t getRandomStatesEnd()
  {
    return ivRandomStates.end();
  }

  /**
   * @return The costs (in mm, truncated as int) to reach the
   * planning state ToStateID from within planning state FromStateID.
   */
  int GetFromToHeuristic(int FromStateID, int ToStateID);

  /**
   * @return The heuristic value to reach the goal from within the
   * planning state stateID (used for forward planning).
   */
  int GetGoalHeuristic(int stateID);

  /**
   * @return The heuristic value to reach the start from within
   * the planning state stateID. (Used for backward planning.)
   */
  int GetStartHeuristic(int stateID);

  /**
   * @brief Calculates the successor states and the corresponding costs
   * when performing the footstep set on the planning state SourceStateID.
   * (Used for forward planning.)
   */
  void GetSuccs(int SourceStateID, std::vector<int> *SuccIDV, std::vector<int> *CostV);

  /**
   * @brief Calculates the predecessor states and the corresponding costs
   * when reversing the footstep set on the planning state TargetStateID.
   * (Used for backward planning.)
   */
  void GetPreds(int TargetStateID, std::vector<int> *PredIDV, std::vector<int> *CostV);

  /**
   * @brief Used for RStar: generate succs/preds at some
   * domain-dependent distance. The number of generated succs/preds is up
   * to the environment.
   */
  virtual void GetRandomSuccsatDistance(int SourceStateID,
                                        std::vector<int>* SuccIDV,
                                        std::vector<int>* CLowV);

  /**
   * @brief Used for RStar: generate succs/preds at some
   * domain-dependent distance. The number of generated succs/preds is up
   * to the environment.
   */
  virtual void GetRandomPredsatDistance(int TargetStateID,
                                        std::vector<int>* PredIDV,
                                        std::vector<int>* CLowV);

  /// Testing, for R*
  void GetSuccsTo(int SourceStateID, int goalStateID,
                  std::vector<int> *SuccIDV, std::vector<int> *CostV);

  /// @return True if two states meet the same condition. Used for R*.
  bool AreEquivalent(int StateID1, int StateID2);

  bool InitializeEnv(const char *sEnvFile);

  bool InitializeMDPCfg(MDPConfig *MDPCfg);

  void PrintEnv_Config(FILE *fOut);

  void PrintState(int stateID, bool bVerbose, FILE *fOut);

  void SetAllActionsandAllOutcomes(CMDPSTATE *state);

  void SetAllPreds(CMDPSTATE *state);

  int SizeofCreatedEnv();

  /**
   * @return True iff 'to' can be reached by an arbitrary footstep that
   * can be performed by the robot from within 'from'. This check is based
   * on given step range polygon in config. (This method is used to check
   * whether the goal/start can be reached from within the current state.)
   */
  bool reachable(const PlanningState& from, const PlanningState& to);

  /**
   * @brief Update the heuristic values (e.g. after the map has changed).
   * The environment takes care that the update is only done when it is
   * necessary.
   */
  void updateHeuristicValues();

  /// Used to scale continuous values in meter to discrete values in mm.
  static const int cvMmScale = 1000;

protected:
  /**
   * @return The costs (in mm, truncated as int) to reach the
   * planning state ToStateID from within planning state FromStateID.
   */
  int GetFromToHeuristic(const PlanningState& from, const PlanningState& to);

  /// @return computes step cost based on the swing_foot from 'before' to 'after' while standing on stand_foot
  int getStepCost(const State& stand_foot, const State& swing_foot_before, const State& swing_foot_after, int &risk_cost);
  int getStepCost(const State &stand_foot, const State &swing_foot_before, const State &swing_foot_after);

  void GetRandomNeighs(const PlanningState* currentState,
                       std::vector<int>* NeighIDV,
                       std::vector<int>* CLowV,
                       int nNumofNeighs, int nDist_c, bool bSuccs);

  void setStateArea(const PlanningState& left, const PlanningState& right);

  /// Wrapper for FootstepPlannerEnvironment::createNewHashEntry(PlanningState).
  PlanningState* createNewHashEntry(const State& s);

  /**
   * @brief Creates a new planning state for 's' and inserts it into the
   * maps (PlanningState::ivStateId2State,
   * PlanningState::ivpStateHash2State)
   *
   * @return A pointer to the newly created PlanningState.
   */
  PlanningState *createNewHashEntry(const PlanningState& s);

  /// Wrapper for FootstepPlannerEnvironment::getHashEntry(PlanningState).
  PlanningState* getHashEntry(const State& s);

  /**
   * @return The pointer to the planning state 's' stored in
   * FootstepPlannerEnvironment::ivpStateHash2State.
   */
  PlanningState* getHashEntry(const PlanningState& s);

  PlanningState *createHashEntryIfNotExists(const PlanningState& s);

  /**
   * @return True iff 'goal' can be reached by an arbitrary footstep.
   * (Used for forward planning.)
   */
  bool closeToGoal(const PlanningState& from);

  /**
   * @return True iff 'start' can be reached by an arbitrary footstep.
   * (Used for backward planning.)
   */
  bool closeToStart(const PlanningState& from);

  /// < operator for planning states.
  struct less
  {
    bool operator ()(const PlanningState* a,
                     const PlanningState* b) const;
  };

  /// Parameters
  const environment_params& ivParams;

  /// ID of Start and Goal foot chosen by planner
  int ivIdPlanningStart;
  int ivIdPlanningGoal;

  /// ID of the start pose of the left foot.
  int ivIdStartFootLeft;
  /// ID of the start pose of the right foot.
  int ivIdStartFootRight;
  /// ID of the goal pose of the left foot.
  int ivIdGoalFootLeft;
  /// ID of the goal pose of the right foot.
  int ivIdGoalFootRight;

  std::vector<int> ivStateArea;

  /**
   * @brief Maps from an ID to the corresponding PlanningState. (Used in
   * the SBPL to access a certain PlanningState.)
   */
  std::vector<const PlanningState*> ivStateId2State;

  /**
   * @brief Maps from a hash tag to a list of corresponding planning
   * states. (Used in FootstepPlannerEnvironment to identify a certain
   * PlanningState.)
   */
  std::vector<PlanningState*>* ivpStateHash2State;

  /// The set of footsteps used for the path planning.
  const std::vector<Footstep>& ivPreDefFootstepSet;

  /// The set of footsteps used for GPR.
  std::vector<Footstep> ivContFootstepSet;

  /// The step cost estimator used by the planner.
  StepCostEstimator::Ptr ivStepCostEstimatoPtr;

  /// The heuristic function used by the planner.
  boost::shared_ptr<Heuristic> ivHeuristicPtr;

  /// terrain model
  TerrainModel::Ptr ivTerrainModel;

  /// The maximal translation in x, y direction (based on step range polygon and discretized in cell size).
  const int ivMaxStepRangeX;
  const int ivMaxStepRangeY;
  /// The maximal rotation (discretized into bins).
  int ivMaxStepRangeTheta;

  /// The minimal translation in x, y direction (based on step range polygon and discretized in cell size).
  const int ivMaxInvStepRangeX;
  const int ivMaxInvStepRangeY;
  /// The minimal rotation (discretized into bins).
  int ivMaxInvStepRangeTheta;

  /// The maximal step range based on given step range polygon
  double ivMaxStepRangeWidth;

  /// The global max step distance based on the step range polygon and footstep set
  double ivMaxStepDist;

  /// foot area in [mm^2]
  const double ivFootArea;

  /**
   * @brief The default costs for each step (discretized with the help of
   * cvMmScale).
   */
  const int ivDefaultStepCost;

  /// defines planning of more detailed parameters (e.g. lift height)
  uint8_t ivPlannningMode;

  /// distance of random neighbors for R* (discretized in cells)
  const int ivRandomNodeDist;

  /// Indicates if heuristic has to be updated.
  bool ivHeuristicExpired;

  /// Pointer to the maps.
  boost::shared_ptr<gridmap_2d::GridMap2D> ivGroundLevelMapPtr;
  boost::shared_ptr<gridmap_2d::GridMap2D> ivBodyLevelMapPtr;

  exp_states_2d_t ivExpandedStates;
  exp_states_t ivRandomStates;  ///< random intermediate states for R*
  size_t ivNumExpandedStates;

  bool* ivpStepRange;
};
}

#endif  // FOOTSTEP_PLANNER_FOOTSTEPPLANNERENVIRONMENT_H_
