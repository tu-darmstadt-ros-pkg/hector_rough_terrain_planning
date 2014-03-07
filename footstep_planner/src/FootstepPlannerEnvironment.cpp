// SVN $HeadURL: http://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_stacks/humanoid_navigation/footstep_planner/src/FootstepPlannerEnvironment.cpp $
// SVN $Id: FootstepPlannerEnvironment.cpp 4168 2013-05-21 09:55:23Z garimort@informatik.uni-freiburg.de $

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

#include <footstep_planner/FootstepPlannerEnvironment.h>

#include <visualization_msgs/MarkerArray.h>


namespace footstep_planner
{
FootstepPlannerEnvironment::FootstepPlannerEnvironment(
    const environment_params& params)
: DiscreteSpaceInformation(),
  ivParams(params),
  ivIdPlanningStart(-1),
  ivIdPlanningGoal(-1),
  ivIdStartFootLeft(-1),
  ivIdStartFootRight(-1),
  ivIdGoalFootLeft(-1),
  ivIdGoalFootRight(-1),
  ivpStateHash2State(new std::vector<PlanningState*>[params.hash_table_size]),
  ivPreDefFootstepSet(params.footstep_set),
  ivMaxStepRangeX(disc_val(params.max_step_range_x, params.cell_size)),
  ivMaxStepRangeY(disc_val(params.max_step_range_y, params.cell_size)),
  ivMaxStepRangeTheta(angle_state_2_cell(params.max_step_range_theta, params.angle_bin_size)),
  ivMaxInvStepRangeX(disc_val(params.max_inverse_step_range_x, params.cell_size)),
  ivMaxInvStepRangeY(disc_val(params.max_inverse_step_range_y, params.cell_size)),
  ivMaxInvStepRangeTheta(angle_state_2_cell(params.max_inverse_step_range_theta, params.angle_bin_size)),
  ivMaxStepRangeWidth(double(disc_val(params.max_step_range_width, params.cell_size))),
  ivMaxStepDist(double(disc_val(params.max_step_dist, params.cell_size))),
  ivFootArea(params.foot_size.x * params.foot_size.y),
  ivDefaultStepCost(cvMmScale * params.const_step_cost),
  ivRandomNodeDist(params.random_node_distance / params.cell_size),
  ivHeuristicExpired(true),
  ivNumExpandedStates(0)
{
  ivStepCostEstimatoPtr = initStepCostEstimator();
  ivHeuristicPtr = initHeuristic();

  ivTerrainModel.reset();

  int num_angle_bins_half = ivParams.num_angle_bins / 2;
  if (ivMaxStepRangeTheta >= num_angle_bins_half)
    ivMaxStepRangeTheta -= ivParams.num_angle_bins;
  if (ivMaxInvStepRangeTheta >= num_angle_bins_half)
    ivMaxInvStepRangeTheta -= ivParams.num_angle_bins;

  int num_x = ivMaxStepRangeX - ivMaxInvStepRangeX + 1;
  ivpStepRange = new bool[num_x * (ivMaxStepRangeY - ivMaxInvStepRangeY + 1)];

  // determine whether a (x,y) translation can be performed by the robot by
  // checking if it is within a certain area of performable steps
  for (int y = ivMaxInvStepRangeY; y <= ivMaxStepRangeY; ++y)
  {
    for (int x = ivMaxInvStepRangeX; x <= ivMaxStepRangeX; ++x)
    {
      bool in_step_range = pointWithinPolygon(x, y, params.step_range);
      ivpStepRange[(y - ivMaxInvStepRangeY) * num_x + (x - ivMaxInvStepRangeX)] = in_step_range;

      if (!in_step_range)
        continue;

      // generate area of samplings for gpr/map-based planning
      for (int theta = ivMaxInvStepRangeTheta; theta <= ivMaxStepRangeTheta; ++theta)
      {
        Footstep f(cont_val(x, ivParams.cell_size), cont_val(y, ivParams.cell_size), angle_cell_2_state(theta, ivParams.angle_bin_size),
                   ivParams.swing_height,
                   ivParams.lift_height,
                   ivParams.step_duration,
                   ivParams.sway_duration,
                   0.0,
                   ivParams.cell_size,
                   ivParams.num_angle_bins,
                   ivParams.hash_table_size);
        ivContFootstepSet.push_back(f);
      }
    }
  }

  // tests
  /*ros::NodeHandle nh;
  ivTestPub = nh.advertise<visualization_msgs::MarkerArray>("test", 1, true);

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "/world";
  marker.ns = "test";
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;

  marker.id = 0;

  PlanningState stand (  0.0,  0.0, 0.0, 0.0, 0.0, 0.0, ivParams.swing_height, ivParams.step_duration, RIGHT, ivParams.cell_size, ivParams.angle_bin_size, ivParams.hash_table_size);
  PlanningState before(-0.20, 0.25, 0.0, 0.0, 0.0, 0.0, ivParams.swing_height, ivParams.step_duration,  LEFT, ivParams.cell_size, ivParams.angle_bin_size, ivParams.hash_table_size);

  for(std::vector<Footstep>::const_iterator iter = ivGPRFootstepSet.begin(); iter != ivGPRFootstepSet.end(); iter++)
  {
    const PlanningState after = iter->performMeOnThisState(stand);


    //ROS_INFO("%i %i %i", after.getX(), after.getY(), after.getYaw());
    //const State &afterState = after.getState();
    //ROS_INFO("%f %f %f", afterState.getX(), afterState.getY(), afterState.getYaw());


    //ROS_INFO("%f %f %f", after.getX(), after.getY(), after.getYaw());

    int risk_cost;
    int cost = stepCost(stand, before, after, risk_cost);

    if (risk_cost < 500)
    {
      marker.id++;

      tf::poseTFToMsg(after.getState().getPose(), marker.pose);
      marker.pose.position.z = after.getState().getYaw();

      marker.color.r = double(risk_cost)/cvMmScale;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;
      marker_array.markers.push_back(marker);
    }
  }
  ivTestPub.publish(marker_array);*/

  /*ROS_INFO("x: [%i %i], y: [%i %i], theta: [%i %i]",
            ivMaxInvStepRangeX, ivMaxStepRangeX,
            ivMaxInvStepRangeY, ivMaxStepRangeY,
            ivMaxInvStepRangeTheta, ivMaxStepRangeTheta);
  std::string msg;
  for (int x = ivMaxStepRangeX; x >= ivMaxInvStepRangeX; --x)
  {
    //msg += boost::lexical_cast<std::string>(x) + ": ";
    for (int y = ivMaxStepRangeY; y >= ivMaxInvStepRangeY; --y)
    {
      bool in_step_range = ivpStepRange[(y - ivMaxInvStepRangeY) * num_x + (x - ivMaxInvStepRangeX)];
      msg += in_step_range ? "+ " : "- ";
    }

    ROS_INFO("%s", msg.c_str());
    msg.clear();
  }
  ROS_INFO("Size: %lu of %u", ivContFootstepSet.size(), num_x * (ivMaxStepRangeY - ivMaxInvStepRangeY + 1) * (ivMaxStepRangeTheta - ivMaxInvStepRangeTheta + 1));
  ROS_INFO("%f %f %i", params.max_inverse_step_range_theta, params.max_step_range_theta, params.num_angle_bins);*/
}

FootstepPlannerEnvironment::~FootstepPlannerEnvironment()
{
  reset();
  if (ivpStateHash2State)
  {
    delete[] ivpStateHash2State;
    ivpStateHash2State = NULL;
  }
  if (ivpStepRange)
  {
    delete[] ivpStepRange;
    ivpStepRange = NULL;
  }
}


StepCostEstimator::Ptr FootstepPlannerEnvironment::initStepCostEstimator()
{
  StepCostEstimator::Ptr const_step_cost_estimator = StepCostEstimator::Ptr(new ConstStepCostEstimator(ivParams.const_step_cost));
  StepCostEstimator::Ptr euclidean_step_cost_estimator = StepCostEstimator::Ptr(new EuclideanStepCostEstimator(const_step_cost_estimator));

  StepCostEstimator::Ptr estimator;
  if (ivParams.step_cost_estimator_type_name == "ConstStepCostEstimator")
  {
    estimator = const_step_cost_estimator;
  }
  else if (ivParams.step_cost_estimator_type_name == "EuclideanStepCostEstimator")
  {
    estimator = euclidean_step_cost_estimator;
  }
  else if (ivParams.step_cost_estimator_type_name == "GprStepCostEstimator")
  {
    estimator.reset(new GprStepCostEstimator(euclidean_step_cost_estimator, ivParams.gpr_filename));
  }
  else if (ivParams.step_cost_estimator_type_name == "MapStepCostEstimator")
  {
    estimator.reset(new MapStepCostEstimator(const_step_cost_estimator, ivParams.map_step_cost_filename));
  }
  else if (ivParams.step_cost_estimator_type_name == "BoundaryStepCostEstimator")
  {
    estimator.reset(new BoundaryStepCostEstimator(euclidean_step_cost_estimator,
                                                  ivParams.boundary_est_max_diff_z,
                                                  ivParams.boundary_est_long_step_dist,
                                                  ivParams.boundary_est_min_yaw_seperation_enlargement,
                                                  ivParams.boundary_est_yaw_enlarged_min_seperation,
                                                  ivParams.boundary_est_cost_roll_abs,
                                                  ivParams.boundary_est_cost_pitch_abs,
                                                  ivParams.boundary_est_cost_yaw_rel,
                                                  ivParams.boundary_est_cost_height_diff_rel));
  }
  else if (ivParams.step_cost_estimator_type_name == "DynamicsStepCostEstimator")
  {
    StepCostEstimator::Ptr boundary_step_cost_estimator = StepCostEstimator::Ptr(
                                                          new BoundaryStepCostEstimator(euclidean_step_cost_estimator,
                                                                                        ivParams.boundary_est_max_diff_z,
                                                                                        ivParams.boundary_est_long_step_dist,
                                                                                        ivParams.boundary_est_min_yaw_seperation_enlargement,
                                                                                        ivParams.boundary_est_yaw_enlarged_min_seperation,
                                                                                        ivParams.boundary_est_cost_roll_abs,
                                                                                        ivParams.boundary_est_cost_pitch_abs,
                                                                                        ivParams.boundary_est_cost_yaw_rel,
                                                                                        ivParams.boundary_est_cost_height_diff_rel));

    estimator.reset(new DynamicsStepCostEstimator(boundary_step_cost_estimator,
                                                  *this,
                                                  ivParams.dynamic_est_lower_step_limit,
                                                  ivParams.dynamic_est_upper_step_limit,
                                                  ivParams.dynamic_est_max_near_distance));
  }

  // add always ground contact support step cost estimator at top level
  estimator.reset(new GroundContactStepCostEstimator(estimator, ivParams.foot_contact_minimal_support));

  if (!estimator)
  {
    ROS_ERROR_STREAM("Step cost estimtation " << ivParams.step_cost_estimator_type_name << " not available, exiting.");
    exit(1);
  }

  ROS_INFO("FootstepPlanner step cost estimator: %s", estimator->getName());

  return estimator;
}

boost::shared_ptr<Heuristic> FootstepPlannerEnvironment::initHeuristic()
{
  boost::shared_ptr<Heuristic> h;
  if (ivParams.heuristic_type_name == "EuclideanHeuristic")
  {
    h.reset(new EuclideanHeuristic(ivParams.cell_size,
                                   ivParams.num_angle_bins));
    ROS_INFO("FootstepPlanner heuristic: euclidean distance");
  }
  else if (ivParams.heuristic_type_name == "EuclStepCostHeuristic")
  {
    h.reset(new EuclStepCostHeuristic(ivParams.cell_size,
                                      ivParams.num_angle_bins,
                                      ivParams.const_step_cost,
                                      ivParams.diff_angle_cost,
                                      std::max(std::abs(ivParams.max_step_range_x), std::abs(ivParams.max_inverse_step_range_x)),
                                      (std::max(std::abs(ivParams.max_step_range_y), std::abs(ivParams.max_inverse_step_range_y)) - ivParams.foot_seperation) * 0.5));
    ROS_INFO("FootstepPlanner heuristic: euclidean distance with step costs");
  }
  else if (ivParams.heuristic_type_name == "PathCostHeuristic")
  {
    // for heuristic inflation
    double foot_incircle =
      std::min((ivParams.foot_size.x / 2.0 -
                std::abs(ivParams.foot_origin_shift.x)),
               (ivParams.foot_size.y / 2.0 -
                std::abs(ivParams.foot_origin_shift.y)));
    assert(foot_incircle > 0.0);

    h.reset(new PathCostHeuristic(ivParams.cell_size,
                                  ivParams.num_angle_bins,
                                  ivParams.const_step_cost,
                                  ivParams.diff_angle_cost,
                                  ivParams.max_step_dist,
                                  foot_incircle));
    ROS_INFO("FootstepPlanner heuristic: 2D path euclidean distance with step "
             "costs");
  }
  else if (ivParams.heuristic_type_name == "GPRStepCostHeuristic")
  {
    h.reset(new GPRStepCostHeuristic(ivParams.cell_size,
                                     ivParams.num_angle_bins,
                                     ivParams.const_step_cost,
                                     ivParams.diff_angle_cost,
                                     ivParams.max_step_dist));
    ROS_INFO("FootstepPlanner heuristic: gpr cost");
  }
  else
  {
    ROS_ERROR_STREAM("Heuristic " << ivParams.heuristic_type_name << " not available, exiting.");
    exit(1);
  }

  return h;
}


std::pair<int, int> FootstepPlannerEnvironment::updateGoal(const State& foot_left, const State& foot_right)
{
  // keep the old IDs
  int goal_foot_id_left = ivIdGoalFootLeft;
  int goal_foot_id_right = ivIdGoalFootRight;

  // update the states for both feet (if necessary)
  PlanningState* p_foot_left = getHashEntry(foot_left);
  if (p_foot_left == NULL)
    p_foot_left = createNewHashEntry(foot_left);
  PlanningState* p_foot_right = getHashEntry(foot_right);
  if (p_foot_right == NULL)
    p_foot_right = createNewHashEntry(foot_right);
  ivIdGoalFootLeft = p_foot_left->getId();
  ivIdGoalFootRight = p_foot_right->getId();
  // check if everything has been set correctly
  assert(ivIdGoalFootLeft != -1);
  assert(ivIdGoalFootRight != -1);

  p_foot_left->setSuccState(p_foot_right);
  p_foot_right->setSuccState(p_foot_left);
  p_foot_left->setSuccState(p_foot_right);
  p_foot_right->setSuccState(p_foot_left);

  // if using the forward search a change of the goal states involves an
  // update of the heuristic
  if (ivParams.forward_search)
  {
    // check if the goal states have been changed
    if (goal_foot_id_left != ivIdGoalFootLeft || goal_foot_id_right != ivIdGoalFootRight)
    {
      ivHeuristicExpired = true;
      setStateArea(*p_foot_left, *p_foot_right);
    }
  }

  return std::pair<int, int>(ivIdGoalFootLeft, ivIdGoalFootRight);
}


std::pair<int, int> FootstepPlannerEnvironment::updateStart(const State& foot_left, const State& foot_right)
{
  // keep the old IDs
  int start_foot_id_left = ivIdStartFootLeft;
  int start_foot_id_right = ivIdStartFootRight;

  // update the states for both feet (if necessary)
  PlanningState* p_foot_left = getHashEntry(foot_left);
  if (p_foot_left == NULL)
    p_foot_left = createNewHashEntry(foot_left);
  PlanningState* p_foot_right = getHashEntry(foot_right);
  if (p_foot_right == NULL)
    p_foot_right = createNewHashEntry(foot_right);
  ivIdStartFootLeft = p_foot_left->getId();
  ivIdStartFootRight = p_foot_right->getId();
  // check if everything has been set correctly
  assert(ivIdStartFootLeft != -1);
  assert(ivIdStartFootRight != -1);

  p_foot_left->setPredState(p_foot_right);
  p_foot_right->setPredState(p_foot_left);
  p_foot_left->setSuccState(p_foot_right);
  p_foot_right->setSuccState(p_foot_left);

  // if using the backward search a change of the start states involves an
  // update of the heuristic
  if (!ivParams.forward_search)
  {
    // check if the start states have been changed
    if (start_foot_id_left != ivIdStartFootLeft || start_foot_id_right != ivIdStartFootRight)
    {
      ivHeuristicExpired = true;
      setStateArea(*p_foot_left, *p_foot_right);
    }
  }

  return std::pair<int, int>(ivIdStartFootLeft, ivIdStartFootRight);
}

void FootstepPlannerEnvironment::setPlannerStartAndGoal()
{
  State robot_start;
  getStartState(robot_start);

  State robot_goal;
  getGoalState(robot_goal);

  tf::Transform direction = robot_start.getPose().inverse() * robot_goal.getPose();

  if (direction.getOrigin().getY() > 0.0)
  {
    ivIdPlanningStart = ivIdStartFootRight;
    ivIdPlanningGoal = ivIdGoalFootLeft;
  }
  else
  {
    ivIdPlanningStart = ivIdStartFootLeft;
    ivIdPlanningGoal = ivIdGoalFootRight;
  }
}


PlanningState *FootstepPlannerEnvironment::createNewHashEntry(const State& s)
{
  PlanningState tmp(s, ivParams.cell_size, ivParams.angle_bin_size, ivParams.hash_table_size);
  return createNewHashEntry(tmp);
}


PlanningState* FootstepPlannerEnvironment::createNewHashEntry(const PlanningState& s)
{
  unsigned int state_hash = s.getHashTag();
  PlanningState* new_state = new PlanningState(s);

  size_t state_id = ivStateId2State.size();
  assert(state_id < (size_t)std::numeric_limits<int>::max());

  // insert the ID of the new state into the corresponding map
  new_state->setId(state_id);
  ivStateId2State.push_back(new_state);

  // insert the new state into the hash map at the corresponding position
  ivpStateHash2State[state_hash].push_back(new_state);

  int* entry = new int[NUMOFINDICES_STATEID2IND];
  StateID2IndexMapping.push_back(entry);
  for(int i = 0; i < NUMOFINDICES_STATEID2IND; ++i)
  {
    StateID2IndexMapping[state_id][i] = -1;
  }

  assert(StateID2IndexMapping.size() - 1 == state_id);

  return new_state;
}


PlanningState* FootstepPlannerEnvironment::getHashEntry(const State& s)
{
  PlanningState tmp(s, ivParams.cell_size, ivParams.angle_bin_size, ivParams.hash_table_size);
  return getHashEntry(tmp);
}


PlanningState* FootstepPlannerEnvironment::getHashEntry(const PlanningState& s)
{
  unsigned int state_hash = s.getHashTag();
  std::vector<PlanningState*>::const_iterator state_iter;
  for (state_iter = ivpStateHash2State[state_hash].begin();
       state_iter != ivpStateHash2State[state_hash].end();
       ++state_iter)
  {
    if (*(*state_iter) == s)
      return *state_iter;
  }

  return NULL;
}

PlanningState* FootstepPlannerEnvironment::createHashEntryIfNotExists(const PlanningState& s)
{
  PlanningState* hash_entry = getHashEntry(s);
  if (hash_entry == NULL)
    hash_entry = createNewHashEntry(s);

  return hash_entry;

}


int FootstepPlannerEnvironment::getStepCost(const State &stand_foot,
                                            const State &swing_foot_before,
                                            const State &swing_foot_after,
                                            int &risk_cost)
{
  if (swing_foot_before == swing_foot_after)
    return 0;

  if (!ivStepCostEstimatoPtr)
  {
    ROS_ERROR("Can't compute step cost: Step cost estimator not initialized!");
    return 0;
  }

  if (stand_foot.getLeg() == swing_foot_before.getLeg())
  {
    ROS_ERROR("Can't compute step cost: No standing foot is same leg as swing foot!");
    return 0;
  }

  if (swing_foot_before.getLeg() != swing_foot_after.getLeg())
  {
    ROS_ERROR("Can't compute step cost: Swing foot states have not same leg!");
    return 0;
  }

  const State& left_foot  = stand_foot.getLeg() == LEFT  ? stand_foot : swing_foot_before;
  const State& right_foot = stand_foot.getLeg() == RIGHT ? stand_foot : swing_foot_before;

  double risk = 0.0;
  double step_cost = ivStepCostEstimatoPtr->getCost(left_foot, right_foot, swing_foot_after, risk);

  risk_cost = cvMmScale * risk;
  return cvMmScale * (risk + step_cost);
}


int FootstepPlannerEnvironment::getStepCost(const State& stand_foot, const State& swing_foot_before, const State& swing_foot_after)
{
  int risk_cost = 0;
  return getStepCost(stand_foot, swing_foot_before, swing_foot_after, risk_cost);
}


bool FootstepPlannerEnvironment::occupiedUpperBody(const State& left, const State& right)
{
  if (!ivBodyLevelMapPtr)
  {
    ROS_ERROR_THROTTLE(10, "No body level grid map available yet.");
    return false;
  }

  // approximate upper body dimensions
  float x = right.getX() + 0.5 * (left.getX() - right.getX());
  float y = right.getY() + 0.5 * (left.getY() - right.getY());

  // collision check for the planning state
  if (ivBodyLevelMapPtr->isOccupiedAt(x, y))
    return true;

  float theta = right.getYaw() + 0.5 * (left.getYaw() - right.getYaw());

  // determine shift of polygon based on foot orientation
  float cos_theta = cos(theta);
  float sin_theta = sin(theta);
  float shift_x = cos_theta * ivParams.upper_body_origin_shift.x - sin_theta * ivParams.upper_body_origin_shift.y;
  float shift_y = sin_theta * ivParams.upper_body_origin_shift.x + cos_theta * ivParams.upper_body_origin_shift.y;

  // shift pose
  x += shift_x;
  y += shift_y;

  return collision_check(x, y, cos_theta, sin_theta, ivParams.upper_body_size.x, ivParams.upper_body_size.y, ivParams.collision_check_accuracy, *ivBodyLevelMapPtr);
}

bool FootstepPlannerEnvironment::occupiedFoot(const State& s)
{
  if (!ivGroundLevelMapPtr)
  {
    ROS_ERROR_THROTTLE(10, "No ground level grid map available yet.");
    return false;
  }

  double x = s.getX();
  double y = s.getY();

  // collision check for the planning state (holds for support area checks too)
  if (ivGroundLevelMapPtr->isOccupiedAt(x,y))
    return true;

  double theta = s.getYaw();

  // collision check for the foot center
  return collision_check(x, y, cos(theta), sin(theta), ivParams.foot_size.x, ivParams.foot_size.y, ivParams.collision_check_accuracy, *ivGroundLevelMapPtr);
}


bool FootstepPlannerEnvironment::getState(unsigned int id, State &s) const
{
  if (id >= ivStateId2State.size())
    return false;

  s = ivStateId2State[id]->getState();
  return true;
}

void FootstepPlannerEnvironment::getStartState(State &left, State &right) const
{
  left = ivStateId2State[ivIdStartFootLeft]->getState();
  right = ivStateId2State[ivIdStartFootRight]->getState();
}

void FootstepPlannerEnvironment::getStartState(State &robot) const
{
  const State &left = ivStateId2State[ivIdStartFootLeft]->getState();
  const State &right = ivStateId2State[ivIdStartFootRight]->getState();

  robot.setX(0.5*(left.getX()+right.getX()));
  robot.setY(0.5*(left.getY()+right.getY()));
  robot.setZ(0.5*(left.getZ()+right.getZ()));
  robot.setYaw(0.5*(left.getYaw()+right.getYaw()));
}

void FootstepPlannerEnvironment::getGoalState(State &left, State &right) const
{
  left = ivStateId2State[ivIdGoalFootLeft]->getState();
  right = ivStateId2State[ivIdGoalFootRight]->getState();
}

void FootstepPlannerEnvironment::getGoalState(State &robot) const
{
  const State &left = ivStateId2State[ivIdGoalFootLeft]->getState();
  const State &right = ivStateId2State[ivIdGoalFootRight]->getState();

  robot.setX(0.5*(left.getX()+right.getX()));
  robot.setY(0.5*(left.getY()+right.getY()));
  robot.setZ(0.5*(left.getZ()+right.getZ()));
  robot.setYaw(0.5*(left.getYaw()+right.getYaw()));
}


void FootstepPlannerEnvironment::updateGroundLevelMap(gridmap_2d::GridMap2DPtr map)
{
  ivGroundLevelMapPtr.reset();
  ivGroundLevelMapPtr = map;

  if (ivHeuristicPtr->getHeuristicType() == Heuristic::PATH_STEP_COST_HEURISTIC)
  {
    boost::shared_ptr<PathCostHeuristic> h = boost::dynamic_pointer_cast<PathCostHeuristic>(ivHeuristicPtr);
    h->updateMap(map);
  }
}

void FootstepPlannerEnvironment::updateBodyLevelMap(gridmap_2d::GridMap2DPtr map)
{
  ivBodyLevelMapPtr.reset();
  ivBodyLevelMapPtr = map;

  /// @TODO
//  if (ivHeuristicConstPtr->getHeuristicType() == Heuristic::PATH_COST)
//  {
//    boost::shared_ptr<PathCostHeuristic> h =
//        boost::dynamic_pointer_cast<PathCostHeuristic>(
//            ivHeuristicConstPtr);
//    h->updateMap(map);
//  }
}


void FootstepPlannerEnvironment::updateTerrainModel(const flor_terrain_classifier::TerrainModel::ConstPtr& terrain_model)
{
  ivTerrainModel.reset(new TerrainModel(terrain_model, ivParams.cell_size, ivParams.foot_size,
                                        ivParams.foot_contact_min_sampling_steps_x, ivParams.foot_contact_min_sampling_steps_y,
                                        ivParams.foot_contact_max_sampling_steps_x, ivParams.foot_contact_max_sampling_steps_y,
                                        ivParams.foot_contact_max_intrusion_z, ivParams.foot_contact_max_ground_clearance));
}


void FootstepPlannerEnvironment::updateHeuristicValues()
{
  // check if start and goal have been set
  assert(ivIdGoalFootLeft != -1 && ivIdGoalFootRight != -1);
  assert(ivIdStartFootLeft != -1 && ivIdStartFootRight != -1);

  if (!ivHeuristicExpired)
    return;

  ROS_INFO("Updating the heuristic values.");

  if (ivHeuristicPtr->getHeuristicType() == Heuristic::PATH_STEP_COST_HEURISTIC)
  {
    boost::shared_ptr<PathCostHeuristic> h =
        boost::dynamic_pointer_cast<PathCostHeuristic>(
            ivHeuristicPtr);
    MDPConfig MDPCfg;
    InitializeMDPCfg(&MDPCfg);
    const PlanningState* start = ivStateId2State[MDPCfg.startstateid];
    const PlanningState* goal = ivStateId2State[MDPCfg.goalstateid];

    // NOTE: start/goal state are set to left leg
    bool success;
    if (ivParams.forward_search)
      success = h->calculateDistances(*start, *goal);
    else
      success = h->calculateDistances(*goal, *start);
    if (!success)
    {
      ROS_ERROR("Failed to calculate path cost heuristic.");
      exit(1);
    }
  }

  ROS_DEBUG("Finished updating the heuristic values.");
  ivHeuristicExpired = false;
}


void FootstepPlannerEnvironment::reset()
{
  for(unsigned int i = 0; i < ivStateId2State.size(); ++i)
  {
    if (ivStateId2State[i])
    {
      delete ivStateId2State[i];
    }
  }
  ivStateId2State.clear();

  if (ivpStateHash2State)
  {
    for(int i = 0; i < ivParams.hash_table_size; ++i)
      ivpStateHash2State[i].clear();
  }

  StateID2IndexMapping.clear();

  ivExpandedStates.clear();
  ivNumExpandedStates = 0;
  ivRandomStates.clear();

  ivIdPlanningStart = -1;
  ivIdPlanningGoal = -1;

  ivIdGoalFootLeft = -1;
  ivIdGoalFootRight = -1;
  ivIdStartFootLeft = -1;
  ivIdStartFootRight = -1;

  ivHeuristicExpired = true;
}


bool FootstepPlannerEnvironment::closeToStart(const PlanningState& from)
{
  return reachable(*ivStateId2State[ivIdPlanningStart], from);
}

bool FootstepPlannerEnvironment::closeToGoal(const PlanningState& from)
{
  return reachable(from, *ivStateId2State[ivIdPlanningGoal]);
}

bool FootstepPlannerEnvironment::reachable(const PlanningState& from, const PlanningState& to)
{
  if (from.getLeg() == to.getLeg())
    return false;

  if (euclidean_distance(from.getX(), from.getY(), to.getX(), to.getY()) > ivMaxStepRangeWidth)
    return false;

  tf::Transform step = from.getState().getPose().inverse() * to.getState().getPose();
  int footstep_x = disc_val(step.getOrigin().x(), ivParams.cell_size);
  int footstep_y = disc_val(step.getOrigin().y(), ivParams.cell_size);

  // calculate the footstep rotation
  int footstep_theta = to.getYaw() - from.getYaw();
  // transform the value into [-ivParams.num_angle_bins/2..ivParams.num_angle_bins/2)
  int num_angle_bins_half = ivParams.num_angle_bins / 2;
  if (footstep_theta >= num_angle_bins_half)
    footstep_theta -= ivParams.num_angle_bins;
  else if (footstep_theta < -num_angle_bins_half)
    footstep_theta += ivParams.num_angle_bins;

  // adjust for the left foot
  if (from.getLeg() == LEFT)
  {
    footstep_y = -footstep_y;
    footstep_theta = -footstep_theta;
  }

  // check if footstep_x is not within the executable range
  if (footstep_x > ivMaxStepRangeX || footstep_x < ivMaxInvStepRangeX)
    return false;
  // check if footstep_y is not within the executable range
  if (footstep_y > ivMaxStepRangeY || footstep_y < ivMaxInvStepRangeY)
    return false;
  // check if footstep_theta is not within the executable range
  if (footstep_theta > ivMaxStepRangeTheta || footstep_theta < ivMaxInvStepRangeTheta)
    return false;
  if (!ivpStepRange[(footstep_y - ivMaxInvStepRangeY) * (ivMaxStepRangeX - ivMaxInvStepRangeX + 1) +
                    (footstep_x - ivMaxInvStepRangeX)])
    return false;

  return true;
}


int FootstepPlannerEnvironment::GetFromToHeuristic(int FromStateID, int ToStateID)
{
  assert(FromStateID >= 0 && (unsigned int) FromStateID < ivStateId2State.size());
  assert(ToStateID >= 0 && (unsigned int) ToStateID < ivStateId2State.size());

  if ((FromStateID == ivIdGoalFootLeft && ToStateID == ivIdGoalFootRight)
      || (FromStateID == ivIdGoalFootRight && ToStateID == ivIdGoalFootLeft)){
    return 0;
  }

  const PlanningState* from = ivStateId2State[FromStateID];
  const PlanningState* to = ivStateId2State[ToStateID];
  //    	if (ivHeuristicConstPtr->getHeuristicType() == Heuristic::PATH_COST){
  //    		boost::shared_ptr<PathCostHeuristic> pathCostHeuristic = boost::dynamic_pointer_cast<PathCostHeuristic>(ivHeuristicConstPtr);
  //    		pathCostHeuristic->calculateDistances(*from, *to);
  //    	}
  return GetFromToHeuristic(*from, *to);
}

int FootstepPlannerEnvironment::GetFromToHeuristic(const PlanningState& from, const PlanningState& to)
{
  return cvMmScale * ivParams.heuristic_scale * ivHeuristicPtr->getHValue(from, to);
}


int FootstepPlannerEnvironment::GetGoalHeuristic(int stateID)
{
  const PlanningState* current = ivStateId2State[stateID];
  if (current->getLeg() == LEFT)
    return GetFromToHeuristic(stateID, ivIdGoalFootLeft);
  else
   return GetFromToHeuristic(stateID, ivIdGoalFootRight);
  //return GetFromToHeuristic(stateID, ivIdPlanningGoal);
}


void FootstepPlannerEnvironment::GetPreds(int TargetStateID, std::vector<int> *PredIDV, std::vector<int> *CostV)
{
  PredIDV->clear();
  CostV->clear();

  assert(TargetStateID >= 0 && (unsigned int) TargetStateID < ivStateId2State.size());

  // make start states always absorbing
  if (TargetStateID == ivIdStartFootLeft || TargetStateID == ivIdStartFootRight)
    return;

  const PlanningState* current = ivStateId2State[TargetStateID];

  if (!current->getSuccState())
  {
    ROS_WARN("Step cost should be evaluated but current state has no successor!");
    return;
  }

  // make sure goal state transitions are consistent with
  // GetSuccs(some_state, goal_state) where goal_state is reachable by an
  // arbitrary step from some_state
  if (ivParams.forward_search)
  {
    if (ivStateArea.empty())
      ROS_WARN("This should not happen! Reactivate setStateArea");
    if (TargetStateID == ivIdGoalFootLeft || TargetStateID == ivIdGoalFootRight)
    {
      const PlanningState* s;
      int cost;
      std::vector<int>::const_iterator state_id_iter;
      for(state_id_iter = ivStateArea.begin(); state_id_iter != ivStateArea.end(); ++state_id_iter)
      {
        s = ivStateId2State[*state_id_iter];
        if (s->getLeg() == current->getLeg())
          continue;
        if (*(current->getSuccState()) == *s)
          continue;

        cost = getStepCost(current->getState(), s->getState(), current->getSuccState()->getState());
        PredIDV->push_back(s->getId());
        CostV->push_back(cost);
      }
      return;
    }
  }

  ivExpandedStates.insert(std::pair<int,int>(current->getX(), current->getY()));
  ++ivNumExpandedStates;

  // check if start is reachable
  if (closeToStart(*current))
  {
    const PlanningState* start = ivStateId2State[ivIdPlanningStart];
    if (*(current->getSuccState()) == *start)
      return;

    bool collide = false;
    if ((ivParams.collision_check_type & CHECK_UPPER_BODY) && occupiedUpperBody(current->getState(), start->getState()))
      collide = true;

    if (!collide)
    {
      int risk_cost = 0;
      int cost = getStepCost(current->getState(), start->getState(), current->getSuccState()->getState(), risk_cost);

      /// @TODO: make it to a parameter and dependend from learning mode
      if (risk_cost < 200)
      {
        PredIDV->push_back(start->getId());
        CostV->push_back(cost);
        return;
      }
    }
  }

  // explorate all states
  bool use_predefined_footsteps = true;
  switch (ivStepCostEstimatoPtr->getEstimatorType())
  {
    case StepCostEstimator::GPR_STEP_COST_ESTIMATOR:
    case StepCostEstimator::MAP_STEP_COST_ESTIMATOR:
    case StepCostEstimator::BOUNDARY_STEP_COST_ESTIMATOR:
    case StepCostEstimator::DYNAMICS_STEP_COST_ESTIMATOR:
      use_predefined_footsteps = false;
      break;
    default:
      break;
  }
  const std::vector<Footstep>& footstep_set = use_predefined_footsteps ? ivPreDefFootstepSet : ivContFootstepSet;

  PredIDV->reserve(footstep_set.size());
  CostV->reserve(footstep_set.size());

  for(std::vector<Footstep>::const_iterator footstep_set_iter = footstep_set.begin(); footstep_set_iter != footstep_set.end(); footstep_set_iter++)
  {
    const PlanningState predecessor = footstep_set_iter->reverseMeOnThisState(*current, ivParams.use_terrain_model ? ivTerrainModel : TerrainModel::ConstPtr());

    if (*(current->getSuccState()) == predecessor)
      continue;

    // lookup costs
    int risk_cost = 0;
    int cost = getStepCost(current->getState(), predecessor.getState(), current->getSuccState()->getState(), risk_cost) + footstep_set_iter->getStepCost();

    /// @TODO: make it to a parameter and dependend from learning mode
    if (risk_cost > 200)
      continue;

    // collision check
    if ((ivParams.collision_check_type & FOOT_SUPPORT_AREA) && predecessor.getState().getGroundContactSupport() < ivParams.foot_contact_minimal_support)
      continue;
    if ((ivParams.collision_check_type & CHECK_UPPER_BODY) && occupiedUpperBody(current->getState(), predecessor.getState()))
      continue;
    if ((ivParams.collision_check_type & CHECK_FOOT) && occupiedFoot(predecessor.getState()))
      continue;

    const PlanningState* predecessor_hash = createHashEntryIfNotExists(predecessor);
    PredIDV->push_back(predecessor_hash->getId());
    CostV->push_back(cost);
  }
}


int FootstepPlannerEnvironment::GetStartHeuristic(int stateID)
{
  const PlanningState* current = ivStateId2State[stateID];
  if (current->getLeg() == LEFT)
    return GetFromToHeuristic(stateID, ivIdStartFootLeft);
  else
    return GetFromToHeuristic(stateID, ivIdStartFootRight);
  //return GetFromToHeuristic(stateID, ivIdPlanningStart);
}


void FootstepPlannerEnvironment::GetSuccs(int SourceStateID, std::vector<int> *SuccIDV, std::vector<int> *CostV)
{
  SuccIDV->clear();
  CostV->clear();

  assert(SourceStateID >= 0 && unsigned(SourceStateID) < ivStateId2State.size());

  // make goal states always absorbing
  if (SourceStateID == ivIdGoalFootLeft || SourceStateID == ivIdGoalFootRight)
    return;

  const PlanningState* current = ivStateId2State[SourceStateID];

  if (!current->getPredState())
  {
    ROS_WARN("Step cost should be evaluated but current state has no predecessor!");
    return;
  }

  // make sure start state transitions are consistent with
  // GetPreds(some_state, start_state) where some_state is reachable by an
  // arbitrary step from start_state
  if (!ivParams.forward_search)
  {
    if (ivStateArea.empty())
      ROS_WARN("This should not happen! Reactivate setStateArea");
    if (SourceStateID == ivIdStartFootLeft || SourceStateID == ivIdStartFootRight)
    {
      const PlanningState* s;
      int cost;
      std::vector<int>::const_iterator state_id_iter;
      for(state_id_iter = ivStateArea.begin(); state_id_iter != ivStateArea.end(); ++state_id_iter)
      {
        s = ivStateId2State[*state_id_iter];
        if (s->getLeg() == current->getLeg())
          continue;
        if (*(current->getPredState()) == *s)
          continue;

        cost = getStepCost(current->getState(), current->getPredState()->getState(), s->getState());
        SuccIDV->push_back(s->getId());
        CostV->push_back(cost);
      }
      return;
    }
  }

  ivExpandedStates.insert(std::pair<int,int>(current->getX(), current->getY()));
  ++ivNumExpandedStates;

  // check if goal is reachable
  if (closeToGoal(*current))
  {
    const PlanningState* goal = ivStateId2State[ivIdPlanningGoal];
    if (*(current->getPredState()) == *goal)
      return;

    bool collide = false;
    if ((ivParams.collision_check_type & CHECK_UPPER_BODY) && occupiedUpperBody(current->getState(), goal->getState()))
      collide = true;

    if (!collide)
    {
      int risk_cost = 0;
      int cost = getStepCost(current->getState(), current->getPredState()->getState(), goal->getState(), risk_cost);

      /// @TODO: make it to a parameter and dependend from learning mode
      if (risk_cost < 200)
      {
        SuccIDV->push_back(goal->getId());
        CostV->push_back(cost);
        return;
      }
    }
  }

  // explorate all states
  bool use_predefined_footsteps = true;
  switch (ivStepCostEstimatoPtr->getEstimatorType())
  {
    case StepCostEstimator::GPR_STEP_COST_ESTIMATOR:
    case StepCostEstimator::MAP_STEP_COST_ESTIMATOR:
    case StepCostEstimator::BOUNDARY_STEP_COST_ESTIMATOR:
    case StepCostEstimator::DYNAMICS_STEP_COST_ESTIMATOR:
      use_predefined_footsteps = false;
      break;
    default:
      break;
  }
  const std::vector<Footstep>& footstep_set = use_predefined_footsteps ? ivPreDefFootstepSet : ivContFootstepSet;

  SuccIDV->reserve(footstep_set.size());
  CostV->reserve(footstep_set.size());

  for(std::vector<Footstep>::const_iterator footstep_set_iter = footstep_set.begin(); footstep_set_iter != footstep_set.end(); footstep_set_iter++)
  {
    const PlanningState successor = footstep_set_iter->performMeOnThisState(*current, ivParams.use_terrain_model ? ivTerrainModel : TerrainModel::ConstPtr());

    if (*(current->getPredState()) == successor)
      continue;

    // lookup costs
    int risk_cost = 0;
    int cost = getStepCost(current->getState(), current->getPredState()->getState(), successor.getState(), risk_cost) + footstep_set_iter->getStepCost();

    /// @TODO: make it to a parameter and dependend from learning mode
    if (risk_cost > 200)
      continue;

    // collision check
    if ((ivParams.collision_check_type & FOOT_SUPPORT_AREA) && successor.getState().getGroundContactSupport() < ivParams.foot_contact_minimal_support)
      continue;
    if ((ivParams.collision_check_type & CHECK_UPPER_BODY) && occupiedUpperBody(current->getState(), successor.getState()))
      continue;
    if ((ivParams.collision_check_type & CHECK_FOOT) && occupiedFoot(successor.getState()))
      continue;

    const PlanningState* successor_hash = createHashEntryIfNotExists(successor);
    SuccIDV->push_back(successor_hash->getId());
    CostV->push_back(cost);
  }
}

void FootstepPlannerEnvironment::GetSuccsTo(int SourceStateID, int goalStateId, std::vector<int> *SuccIDV, std::vector<int> *CostV)
{
  //return GetSuccs(SourceStateID, SuccIDV, CostV);

  SuccIDV->clear();
  CostV->clear();

  assert(SourceStateID >= 0 && unsigned(SourceStateID) < ivStateId2State.size());

  // make goal state absorbing
  if (SourceStateID == ivIdGoalFootLeft ){
    return;
  }

  const PlanningState* current = ivStateId2State[SourceStateID];

  if (!current->getPredState())
  {
    ROS_WARN("Step cost should be evaluated but current state has no predecessor!");
    return;
  }

  ivExpandedStates.insert(std::pair<int,int>(current->getX(), current->getY()));
  ++ivNumExpandedStates;

  //ROS_INFO("GetSuccsTo %d -> %d: %f", SourceStateID, goalStateId, euclidean_distance(current->getX(), current->getY(), ivStateId2State[goalStateId]->getX(), ivStateId2State[goalStateId]->getY()));

  // add cheap transition from right to left, so right becomes an equivalent goal
  if (goalStateId == ivIdGoalFootLeft && SourceStateID == ivIdGoalFootRight && current->getLeg() == RIGHT)
  {
    SuccIDV->push_back(ivIdGoalFootLeft);
    CostV->push_back(ivDefaultStepCost);
    return;
  }

  if (closeToGoal(*current))
  {
    const PlanningState* goal = ivStateId2State[ivIdPlanningGoal];
    if (*(current->getPredState()) == *goal)
      return;

    bool collide = false;
    if ((ivParams.collision_check_type & CHECK_UPPER_BODY) && occupiedUpperBody(current->getState(), goal->getState()))
      collide = true;

    if (!collide)
    {
      int risk_cost = 0;
      int cost = getStepCost(current->getState(), current->getPredState()->getState(), goal->getState());

      /// @TODO: make it to a parameter and dependend from learning mode
      if (risk_cost < 200)
      {
        SuccIDV->push_back(goal->getId());
        CostV->push_back(cost);
        return;
      }
    }
  }

  // intermediate goal reachable (R*)?
  assert(goalStateId >= 0 && unsigned(goalStateId) < ivStateId2State.size());
  const PlanningState* randomGoal = ivStateId2State[goalStateId];
  if (randomGoal->getLeg() != current->getLeg() && *(current->getPredState()) != *randomGoal && reachable(*current, *randomGoal))
  {
    int cost = getStepCost(current->getState(), current->getPredState()->getState(), randomGoal->getState());
    SuccIDV->push_back(goalStateId);
    CostV->push_back(cost);
    //       		ROS_INFO("%d %d", goalStateId, cost);

    //       		return;
  }


  SuccIDV->reserve(ivPreDefFootstepSet.size());
  CostV->reserve(ivPreDefFootstepSet.size());
  std::vector<Footstep>::const_iterator footstep_set_iter;
  for(footstep_set_iter = ivPreDefFootstepSet.begin(); footstep_set_iter != ivPreDefFootstepSet.end(); ++footstep_set_iter)
  {
    PlanningState successor = footstep_set_iter->performMeOnThisState(*current, ivParams.use_terrain_model ? ivTerrainModel : TerrainModel::ConstPtr());
    if (*(current->getPredState()) == successor)
      return;
    if ((ivParams.collision_check_type & FOOT_SUPPORT_AREA) && successor.getState().getGroundContactSupport() < ivParams.foot_contact_minimal_support)
      continue;
    if ((ivParams.collision_check_type & CHECK_UPPER_BODY) && occupiedUpperBody(current->getState(), successor.getState()))
      continue;
    if ((ivParams.collision_check_type & CHECK_FOOT) && occupiedFoot(successor.getState()))
      continue;

    const PlanningState* successor_hash = createHashEntryIfNotExists(successor);

    int cost = getStepCost(current->getState(), current->getPredState()->getState(), successor_hash->getState()) + footstep_set_iter->getStepCost();
    SuccIDV->push_back(successor_hash->getId());
    CostV->push_back(cost);
  }
}


void FootstepPlannerEnvironment::GetRandomSuccsatDistance(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CLowV)
{

  assert(SourceStateID >= 0 && unsigned(SourceStateID) < ivStateId2State.size());
  //goal state should be absorbing
  if (SourceStateID == ivIdGoalFootLeft || SourceStateID == ivIdGoalFootRight )
    return;


  const PlanningState* currentState = ivStateId2State[SourceStateID];
  // TODO: closeToGoal?
  //
  //    	if (closeToGoal(*currentState))
  //    		return;

  //get the successors
  GetRandomNeighs(currentState, SuccIDV, CLowV, ivParams.num_random_nodes,
                  ivRandomNodeDist, true);
}

void FootstepPlannerEnvironment::GetRandomPredsatDistance(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CLowV)
{

  assert(TargetStateID >= 0 &&
                 unsigned(TargetStateID) < ivStateId2State.size());

  // start state should be absorbing
  if (TargetStateID == ivIdStartFootLeft || TargetStateID == ivIdStartFootRight)
    return;

  const PlanningState* currentState = ivStateId2State[TargetStateID];

  // TODO: ???
  //    	if(closeToStart(*currentState))
  //    		return;

  //get the predecessors
  GetRandomNeighs(currentState, PredIDV, CLowV, ivParams.num_random_nodes,
                  ivRandomNodeDist, false);

}

//generates nNumofNeighs random neighbors of cell <X,Y> at distance nDist_c (measured in cells)
//it will also generate goal if within this distance as an additional neighbor
//bSuccs is set to true if we are computing successor states, otherwise it is Preds
// (see fct. implemented in environment_nav2D)
void FootstepPlannerEnvironment::GetRandomNeighs(const PlanningState* currentState, std::vector<int>* NeighIDV, std::vector<int>* CLowV, int nNumofNeighs, int nDist_c, bool bSuccs)
{

  //clear the successor array
  NeighIDV->clear();
  CLowV->clear();


  //get X, Y for the states
  int X = currentState->getX();
  int Y = currentState->getY();
  //int theta = currentState->getTheta();

  //see if the goal/start belongs to the inside area and if yes then add it to Neighs as well
  // NOTE: "goal check" for backward planning
  const PlanningState* goal_left = NULL;
  const PlanningState* goal_right = NULL;
  if (bSuccs){
    goal_left = ivStateId2State[ivIdGoalFootLeft];
    goal_right = ivStateId2State[ivIdGoalFootRight];
  } else {
    goal_left = ivStateId2State[ivIdStartFootLeft];
    goal_right = ivStateId2State[ivIdStartFootRight];
  }

  int nDist_sq = nDist_c*nDist_c;

  //add left if within the distance
  if (euclidean_distance_sq(X, Y, goal_left->getX(), goal_left->getY()) <= nDist_sq)
  {
    //compute clow
    int clow;
    if(bSuccs)
      clow = GetFromToHeuristic(*currentState, *goal_left);
    else
      clow = GetFromToHeuristic(*goal_left, *currentState);

    NeighIDV->push_back(goal_left->getId());
    CLowV->push_back(clow);
    ivRandomStates.push_back(goal_left->getId());
  }

  //add right if within the distance
  if(euclidean_distance_sq(X, Y, goal_right->getX(), goal_right->getY()) <= nDist_sq)
  {
    //compute clow
    int clow;
    if(bSuccs)
      clow = GetFromToHeuristic(*currentState, *goal_right);
    else
      clow = GetFromToHeuristic(*goal_right, *currentState);

    NeighIDV->push_back(goal_right->getId());
    CLowV->push_back(clow);
    ivRandomStates.push_back(goal_right->getId());
  }

  //iterate through random actions
  int nAttempts = 0;
  for (int i = 0; i < nNumofNeighs && nAttempts < 5*nNumofNeighs; ++i, ++nAttempts)
  {

    // pick goal in random direction
    float fDir = (float)(TWO_PI*(((double)rand())/RAND_MAX));

    int dX = (int)(nDist_c*cos(fDir));
    int dY = (int)(nDist_c*sin(fDir));

    //get the coords of the state
    int newX = X + dX;
    int newY = Y + dY;

    // TODO / FIXME x,y, can be negative! need offset
    // check if outside of map:
    //    		if (newX < 0 || newY < 0 || unsigned(newX) >= ivMapPtr->getInfo().width || unsigned(newY) >= ivMapPtr->getInfo().height){
    //    			i--;
    //    			ROS_INFO("Outside of map: %d %d", newX, newY);
    //    			continue;
    //    		}

    // direction of random exploration (facing forward):
    int newYaw = angle_state_2_cell(fDir, ivParams.angle_bin_size);

    // random left/right
    Leg newLeg = Leg(rand() % 2);

    /// @TODO: determine roll and pitch via grid map
    int roll = angle_state_2_cell(0.0, ivParams.angle_bin_size);
    int pitch = angle_state_2_cell(0.0, ivParams.angle_bin_size);
    /// @TODO: determine z, swing/lift height and step/sway duration
    PlanningState randomState(newX, newY, 0, roll, pitch, newYaw, currentState->getSwingHeight(), currentState->getLiftHeight(), currentState->getStepDuration(), currentState->getSwayDuration(), newLeg, ivParams.cell_size, ivParams.angle_bin_size, ivParams.hash_table_size, currentState);
    // add both left and right if available:
    //    		int sep = disc_val(0.07, ivParams.cell_size);
    //    		int ddX = int(-sin(fDir) * sep);
    //    		int ddY = int(cos(fDir) * sep);
    //    		PlanningState randomState(newX+ddX, newY+ddY, newTheta, LEFT, ivParams.hash_table_size);
    //
    //    		PlanningState randomStateR(newX-ddX, newY-ddY, newTheta, RIGHT, ivParams.hash_table_size);

    if(!occupiedFoot(randomState.getState()))
    {
      const PlanningState* random_hash_entry = getHashEntry(randomState);
      if (random_hash_entry == NULL){
        random_hash_entry = createNewHashEntry(randomState);
        ivRandomStates.push_back(random_hash_entry->getId());
      }

      //compute clow
      int clow;
      if(bSuccs)
        clow = GetFromToHeuristic(currentState->getId(), random_hash_entry->getId());

      else
        clow = GetFromToHeuristic(random_hash_entry->getId(), currentState->getId());

      NeighIDV->push_back(random_hash_entry->getId());
      CLowV->push_back(clow);

    }else{
      i--;
    }

    //    		if(!occupied(randomStateR))
    //    		{
    //    			const PlanningState* random_hash_entry = getHashEntry(randomStateR);
    //    			if (random_hash_entry == NULL){
    //    				random_hash_entry = createNewHashEntry(randomStateR);
    //    				ivRandomStates.push_back(random_hash_entry->getId());
    //    			}
    //
    //    			//compute clow
    //    			int clow;
    //    			if(bSuccs)
    //    				clow = GetFromToHeuristic(currentState->getId(), random_hash_entry->getId());
    //    			else
    //    				clow = GetFromToHeuristic(random_hash_entry->getId(), currentState->getId());
    //
    //    			NeighIDV->push_back(random_hash_entry->getId());
    //    			CLowV->push_back(clow);
    //
    //    		}else{
    //    			i--;
    //    		}


  }

  if (NeighIDV->size() == 0){
    ROS_WARN("Could not create any random neighbor nodes (%d attempts) from id %d (%d %d)",
             nAttempts, currentState->getId(), X, Y);
  } else

    ROS_DEBUG("Created %zu random neighbors (%d attempts) from id %d "
        "(%d %d)", NeighIDV->size(), nAttempts, currentState->getId(),
        X, Y);
}

bool
FootstepPlannerEnvironment::AreEquivalent(int StateID1, int StateID2)
{
  assert(StateID1 >= 0 && StateID2 >= 0
         && unsigned(StateID1) < ivStateId2State.size() && unsigned(StateID2) < ivStateId2State.size());


  if (StateID1 == StateID2)
    return true;

  const PlanningState* s1 = ivStateId2State[StateID1];
  const PlanningState* s2 = ivStateId2State[StateID2];

  //		// approximately compare, ignore theta:
  return (std::abs(s1->getX() - s2->getX()) < 1
      && std::abs(s1->getY() - s2->getY()) < 1
      //			                && std::abs(s1->getTheta() - s2->getTheta()) < 3
      && s1->getLeg() == s2->getLeg()
  );


//  compare the actual values (exact comparison)
//  return (*s1 == *s2);
}



bool
FootstepPlannerEnvironment::InitializeEnv(const char *sEnvFile)
{
//  ROS_ERROR("FootstepPlanerEnvironment::InitializeEnv: Hit unimplemented "
//            "function. Check this!");
  return true;
}


bool
FootstepPlannerEnvironment::InitializeMDPCfg(MDPConfig *MDPCfg)
{
  // NOTE: The internal start and goal ids are set here to the left foot
  // (this affects the calculation of the heuristic values)
  MDPCfg->startstateid = ivIdPlanningStart;
  MDPCfg->goalstateid = ivIdPlanningGoal;

  assert(ivIdPlanningStart != -1);
  assert(ivIdPlanningGoal != -1);

  return true;
}


void
FootstepPlannerEnvironment::PrintEnv_Config(FILE *fOut)
{
  // NOTE: implement this if the planner needs to print out configurations
  ROS_ERROR("FootstepPlanerEnvironment::PrintEnv_Config: Hit "
      "unimplemented function. Check this!");
}


void
FootstepPlannerEnvironment::PrintState(int stateID, bool bVerbose,
                                       FILE *fOut)
{
  if(fOut == NULL)
  {
    fOut = stdout;
  }

  if(stateID == ivIdGoalFootLeft && bVerbose)
  {
    SBPL_FPRINTF(fOut, "the state is a goal state\n");
  }

  const PlanningState* s = ivStateId2State[stateID];

  if(bVerbose)
  {
    SBPL_FPRINTF(fOut, "X=%i Y=%i THETA=%i FOOT=%i\n",
                 s->getX(), s->getY(), s->getYaw(), s->getLeg());
  }
  else
  {
    SBPL_FPRINTF(fOut, "%i %i %i %i\n",
                 s->getX(), s->getY(), s->getYaw(), s->getLeg());
  }
}


void
FootstepPlannerEnvironment::SetAllActionsandAllOutcomes(CMDPSTATE *state)
{
  // NOTE: not implemented so far
  // Description: Some searches may also use SetAllActionsandAllOutcomes
  // or SetAllPreds functions if they keep the pointers to successors
  // (predecessors) but most searches do not require this, so it is not
  // necessary to support this

  ROS_ERROR("FootstepPlannerEnvironment::SetAllActionsandAllOutcomes: Hit"
      " unimplemented function. Check this!");
}


void
FootstepPlannerEnvironment::SetAllPreds(CMDPSTATE *state)
{
  // NOTE: not implemented so far
  // Description: Some searches may also use SetAllActionsandAllOutcomes
  // or SetAllPreds functions if they keep the pointers to successors
  // (predecessors) but most searches do not require this, so it is not
  // necessary to support this

  ROS_ERROR("FootstepPlannerEnvironment::SetAllPreds: Hit unimplemented "
      "function. Check this!");
}


int
FootstepPlannerEnvironment::SizeofCreatedEnv()
{
  return ivStateId2State.size();
}


void
FootstepPlannerEnvironment::setStateArea(const PlanningState& left,
                                         const PlanningState& right)
{
  ivStateArea.clear();
  return; /// TODO: disabled, whole stuff may be removed

  const PlanningState* p_state = getHashEntry(right);
  ivStateArea.push_back(p_state->getId());

  double cont_step_x, cont_step_y, cont_step_theta;
  for (int step_y = ivMaxInvStepRangeY; step_y <= ivMaxStepRangeY; ++step_y)
  {
    for (int step_x = ivMaxInvStepRangeX; step_x <= ivMaxStepRangeX; ++step_x)
    {
      for (int step_theta = ivMaxInvStepRangeTheta; step_theta <= ivMaxStepRangeTheta; ++step_theta)
      {
        cont_step_x = cont_val(step_x, ivParams.cell_size);
        cont_step_y = cont_val(step_y, ivParams.cell_size);
        cont_step_theta = angle_cell_2_state(step_theta, ivParams.angle_bin_size);
        Footstep step(cont_step_x, cont_step_y, cont_step_theta, ivParams.swing_height, ivParams.lift_height, ivParams.step_duration, ivParams.sway_duration, 0.0, ivParams.cell_size, ivParams.num_angle_bins, ivParams.hash_table_size);
        if (ivParams.forward_search)
        {
          PlanningState pred = step.reverseMeOnThisState(left);
          if ((ivParams.collision_check_type & FOOT_SUPPORT_AREA) && pred.getState().getGroundContactSupport() < ivParams.foot_contact_minimal_support)
            continue;
          if ((ivParams.collision_check_type & CHECK_UPPER_BODY) && occupiedUpperBody(left.getState(), pred.getState()))
            continue;
          if ((ivParams.collision_check_type & CHECK_FOOT) && occupiedFoot(pred.getState()))
            continue;
          if (!reachable(pred, left))
            continue;

          p_state = createHashEntryIfNotExists(pred);
          ivStateArea.push_back(p_state->getId());

          pred = step.reverseMeOnThisState(right);
          if ((ivParams.collision_check_type & FOOT_SUPPORT_AREA) && pred.getState().getGroundContactSupport() < ivParams.foot_contact_minimal_support)
            continue;
          if ((ivParams.collision_check_type & CHECK_UPPER_BODY) && occupiedUpperBody(right.getState(), pred.getState()))
            continue;
          if ((ivParams.collision_check_type & CHECK_FOOT) && occupiedFoot(pred.getState()))
            continue;
          if (!reachable(pred, right))
            continue;

          p_state = createHashEntryIfNotExists(pred);
          ivStateArea.push_back(p_state->getId());
        }
        else
        {
          PlanningState succ = step.performMeOnThisState(left);
          if ((ivParams.collision_check_type & FOOT_SUPPORT_AREA) && succ.getState().getGroundContactSupport() < ivParams.foot_contact_minimal_support)
            continue;
          if ((ivParams.collision_check_type & CHECK_UPPER_BODY) && occupiedUpperBody(left.getState(), succ.getState()))
            continue;
          if ((ivParams.collision_check_type & CHECK_FOOT) && occupiedFoot(succ.getState()))
            continue;
          if (!reachable(succ, left))
            continue;

          p_state = createHashEntryIfNotExists(succ);
          ivStateArea.push_back(p_state->getId());

          succ = step.performMeOnThisState(right);
          if ((ivParams.collision_check_type & FOOT_SUPPORT_AREA) && succ.getState().getGroundContactSupport() < ivParams.foot_contact_minimal_support)
            continue;
          if ((ivParams.collision_check_type & CHECK_UPPER_BODY) && occupiedUpperBody(right.getState(), succ.getState()))
            continue;
          if ((ivParams.collision_check_type & CHECK_FOOT) && occupiedFoot(succ.getState()))
            continue;
          if (!reachable(succ, right))
            continue;

          p_state = createHashEntryIfNotExists(succ);
          ivStateArea.push_back(p_state->getId());
        }
      }
    }
  }
}


bool
FootstepPlannerEnvironment::less::operator ()(const PlanningState* a,
                                              const PlanningState* b)
const
{
  if (a->getX() < b->getX())
    return true;
  else if (a->getY() < b->getY())
    return true;
  else
    return false;
}
}

