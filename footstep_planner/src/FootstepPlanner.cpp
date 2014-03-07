// SVN $HeadURL: http://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_stacks/humanoid_navigation/footstep_planner/src/FootstepPlanner.cpp $
// SVN $Id: FootstepPlanner.cpp 4170 2013-05-21 11:18:20Z garimort@informatik.uni-freiburg.de $

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

#include <footstep_planner/FootstepPlanner.h>
#include <humanoid_nav_msgs/ClipFootstep.h>


using gridmap_2d::GridMap2D;
using gridmap_2d::GridMap2DPtr;


namespace footstep_planner
{
FootstepPlanner::FootstepPlanner()
  : frame_id("/world")
  , ivStartPoseSetUp(false)
  , ivGoalPoseSetUp(false)
  , ivLastMarkerMsgSize(0)
  , ivPathCost(0)
  , ivShiftGoalPose(false)
  , ivMarkerNamespace("")
  , ivCheckedFootContactSupport(new pcl::PointCloud<pcl::PointXYZI>())
{
  // private NodeHandle for parameters and private messages (debug / info)
  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh_public;

  // ..publishers
  ivExpandedStatesVisPub = nh_private.advertise<sensor_msgs::PointCloud>("expanded_states", 1);
  ivRandomStatesVisPub = nh_private.advertise<sensor_msgs::PointCloud>("random_states", 1);
  ivFootstepPathVisPub = nh_private.advertise<visualization_msgs::MarkerArray>("footsteps_array", 1);
  ivHeuristicPathVisPub = nh_private.advertise<nav_msgs::Path>("heuristic_path", 1);
  ivPathVisPub = nh_private.advertise<nav_msgs::Path>("path", 1);
  ivStartPoseVisPub = nh_private.advertise<geometry_msgs::PoseStamped>("start", 1);
  ivCheckedFootContactSupportPub = nh_private.advertise<sensor_msgs::PointCloud2>("foot_contact_support", 1);
  ivCurrentParamsPub = nh_private.advertise<flor_footstep_planner_msgs::FootstepPlannerParams>("params", 1);

  // used service:
  ivTerrainModelService = nh_public.serviceClient<flor_terrain_classifier::TerrainModelService>("/flor/terrain_classifier/generate_terrain_model");

  // read parameters from config file:
  // planner environment settings
  nh_private.param("step_cost_estimator_type", ivEnvironmentParams.step_cost_estimator_type_name, std::string("EuclideanStepCostEstimator"));
  //nh_private.param("step_cost_type", ivEnvironmentParams.step_cost_type, (int)EUCLIDEAN_STEP_COST);
  nh_private.param("const_step_cost_estimator/step_cost", ivEnvironmentParams.const_step_cost, 0.1);

  nh_private.param("boundary_step_cost_estimator/max_diff_z", ivEnvironmentParams.boundary_est_max_diff_z, 0.4);
  nh_private.param("boundary_step_cost_estimator/long_step_dist", ivEnvironmentParams.boundary_est_long_step_dist, 0.3);
  nh_private.param("boundary_step_cost_estimator/min_yaw_seperation_enlargement", ivEnvironmentParams.boundary_est_min_yaw_seperation_enlargement, 0.2);
  nh_private.param("boundary_step_cost_estimator/yaw_enlarged_min_seperation", ivEnvironmentParams.boundary_est_yaw_enlarged_min_seperation, 0.23);
  nh_private.param("boundary_step_cost_estimator/cost_roll_abs", ivEnvironmentParams.boundary_est_cost_roll_abs, 0.3);
  nh_private.param("boundary_step_cost_estimator/cost_pitch_abs", ivEnvironmentParams.boundary_est_cost_pitch_abs, 0.3);
  nh_private.param("boundary_step_cost_estimator/cost_yaw_rel", ivEnvironmentParams.boundary_est_cost_yaw_rel, 0.2);
  nh_private.param("boundary_step_cost_estimator/cost_height_diff_rel", ivEnvironmentParams.boundary_est_cost_height_diff_rel, 1.0);

  nh_private.param("dynamic_step_cost_estimator/lower_step_limit", ivEnvironmentParams.dynamic_est_lower_step_limit, 0.1);
  nh_private.param("dynamic_step_cost_estimator/upper_step_limit", ivEnvironmentParams.dynamic_est_upper_step_limit, 0.5);
  nh_private.param("dynamic_step_cost_estimator/max_near_distance", ivEnvironmentParams.dynamic_est_max_near_distance, 1.0);

  nh_private.param("post_processing/sway_duration_max_swing_dist", ivEnvironmentParams.post_processing_sw_max_swing_dist, 0.5);
  nh_private.param("post_processing/sway_duration_min_normal_z", ivEnvironmentParams.post_processing_sw_min_normal_z, 0.95);
  nh_private.param("post_processing/sway_duration_adjusted", ivEnvironmentParams.post_processing_sw_adjusted, 0.0);
  nh_private.param("post_processing/toe_off_max_turn_rate", ivEnvironmentParams.post_processing_toe_max_turn_rate, 0.4);
  nh_private.param("post_processing/knee_nominal_max_step_down", ivEnvironmentParams.post_processing_kn_max_step_down, 0.5);
  nh_private.param("post_processing/knee_nominal_adjusted", ivEnvironmentParams.post_processing_kn_adjusted, 0.0);

  nh_private.param("heuristic_type", ivEnvironmentParams.heuristic_type_name, std::string("EuclideanHeuristic"));
  nh_private.param("diff_angle_cost", ivEnvironmentParams.diff_angle_cost, 0.0);
  nh_private.param("heuristic_scale", ivEnvironmentParams.heuristic_scale, 1.0);
  nh_private.param("max_hash_size", ivEnvironmentParams.hash_table_size, 65536);
  nh_private.param("accuracy/collision_check_type", ivEnvironmentParams.collision_check_type, 1);
  nh_private.param("accuracy/collision_check_accuracy", ivEnvironmentParams.collision_check_accuracy, 2);

  nh_private.param("foot_contact_support/min_sampling_steps_x", (int&)ivEnvironmentParams.foot_contact_min_sampling_steps_x, 6);
  nh_private.param("foot_contact_support/min_sampling_steps_y", (int&)ivEnvironmentParams.foot_contact_min_sampling_steps_y, 4);
  nh_private.param("foot_contact_support/max_sampling_steps_x", (int&)ivEnvironmentParams.foot_contact_max_sampling_steps_x, 32);
  nh_private.param("foot_contact_support/max_sampling_steps_y", (int&)ivEnvironmentParams.foot_contact_max_sampling_steps_y, 24);
  nh_private.param("foot_contact_support/max_intrusion_z", ivEnvironmentParams.foot_contact_max_intrusion_z, 0.005);
  nh_private.param("foot_contact_support/max_ground_clearance", ivEnvironmentParams.foot_contact_max_ground_clearance, 0.01);
  nh_private.param("foot_contact_support/minimal_support", ivEnvironmentParams.foot_contact_minimal_support, 0.8);

  nh_private.param("gpr_file", ivEnvironmentParams.gpr_filename, std::string(""));
  nh_private.param("map_step_cost_file", ivEnvironmentParams.map_step_cost_filename, std::string(""));

  nh_private.param("accuracy/cell_size", ivEnvironmentParams.cell_size, 0.01);
  nh_private.param("accuracy/num_angle_bins", ivEnvironmentParams.num_angle_bins, 64);
  ivEnvironmentParams.angle_bin_size = 2.0*M_PI / (double)ivEnvironmentParams.num_angle_bins;

  nh_private.param("planner_type", ivPlannerType, std::string("ARAPlanner"));
  nh_private.param("search_until_first_solution", ivSearchUntilFirstSolution, false);
  nh_private.param("allocated_time", ivMaxSearchTime, 7.0);
  nh_private.param("forward_search", ivEnvironmentParams.forward_search, false);
  nh_private.param("initial_epsilon", ivInitialEpsilon, 3.0);
  nh_private.param("changed_cells_limit", ivChangedCellsLimit, 20000);
  nh_private.param("shift_goal_pose", ivShiftGoalPose, false);
  nh_private.param("num_random_nodes", ivEnvironmentParams.num_random_nodes, 20);
  nh_private.param("random_node_dist", ivEnvironmentParams.random_node_distance, 1.0);

  // foot settings
  nh_private.param("foot/size/x", ivEnvironmentParams.foot_size.x, 0.254);
  nh_private.param("foot/size/y", ivEnvironmentParams.foot_size.y, 0.14);
  nh_private.param("foot/size/z", ivEnvironmentParams.foot_size.z, 0.0508);
  nh_private.param("foot/origin_shift/x", ivEnvironmentParams.foot_origin_shift.x, 0.0);
  nh_private.param("foot/origin_shift/y", ivEnvironmentParams.foot_origin_shift.y, 0.0);
  nh_private.param("foot/origin_shift/z", ivEnvironmentParams.foot_origin_shift.z, 0.0);
  nh_private.param("foot/separation", ivEnvironmentParams.foot_seperation, 0.23);

  // upper body settings
  nh_private.param("upper_body/size/x", ivEnvironmentParams.upper_body_size.x, 0.7);
  nh_private.param("upper_body/size/y", ivEnvironmentParams.upper_body_size.y, 1.1);
  nh_private.param("upper_body/size/z", ivEnvironmentParams.upper_body_size.z, 0.0);
  nh_private.param("upper_body/origin_shift/x", ivEnvironmentParams.upper_body_origin_shift.x, 0.0);
  nh_private.param("upper_body/origin_shift/y", ivEnvironmentParams.upper_body_origin_shift.y, 0.0);
  nh_private.param("upper_body/origin_shift/z", ivEnvironmentParams.upper_body_origin_shift.z, 0.0);

  nh_private.param("swing_height", ivEnvironmentParams.swing_height, 0.15);
  nh_private.param("lift_height", ivEnvironmentParams.lift_height, 0.0);
  nh_private.param("step_duration", ivEnvironmentParams.step_duration, 0.63);
  nh_private.param("sway_duration", ivEnvironmentParams.sway_duration, 0.0);

  nh_private.param("use_terrain_model", ivEnvironmentParams.use_terrain_model, false);



  // load footstep primitives
  XmlRpc::XmlRpcValue footsteps_x;
  XmlRpc::XmlRpcValue footsteps_y;
  XmlRpc::XmlRpcValue footsteps_theta;
  XmlRpc::XmlRpcValue footsteps_swing_height;
  XmlRpc::XmlRpcValue footsteps_step_duration;
  XmlRpc::XmlRpcValue footsteps_step_cost;
  nh_private.getParam("footsteps/x", footsteps_x);
  nh_private.getParam("footsteps/y", footsteps_y);
  nh_private.getParam("footsteps/theta", footsteps_theta);
  nh_private.getParam("footsteps/swing_height", footsteps_swing_height);
  nh_private.getParam("footsteps/step_duration", footsteps_step_duration);
  nh_private.getParam("footsteps/step_cost", footsteps_step_cost);
  if (footsteps_x.getType() != XmlRpc::XmlRpcValue::TypeArray)
    ROS_ERROR("Error reading footsteps/x from config file.");
  if (footsteps_y.getType() != XmlRpc::XmlRpcValue::TypeArray)
    ROS_ERROR("Error reading footsteps/y from config file.");
  if (footsteps_theta.getType() != XmlRpc::XmlRpcValue::TypeArray)
    ROS_ERROR("Error reading footsteps/theta from config file.");
  if (footsteps_swing_height.getType() != XmlRpc::XmlRpcValue::TypeArray)
    ROS_ERROR("Error reading footsteps/swing_height from config file.");
  if (footsteps_step_duration.getType() != XmlRpc::XmlRpcValue::TypeArray)
    ROS_ERROR("Error reading footsteps/step_duration from config file.");
  int size_x = footsteps_x.size();
  int size_y = footsteps_y.size();
  int size_t = footsteps_theta.size();
  int size_s = footsteps_swing_height.size();
  int size_d = footsteps_step_duration.size();
  if (size_x != size_y || size_x != size_t || size_x != size_s || size_x != size_d)
  {
    ROS_ERROR("Footstep parameterization has different sizes for x/y/theta. "
              "Exit!");
    exit(2);
  }
  // create footstep set
  ivEnvironmentParams.footstep_set.clear();
  ivEnvironmentParams.max_step_dist = 0;
  for(int i=0; i < footsteps_x.size(); ++i)
  {
    double x = (double)footsteps_x[i];
    double y = (double)footsteps_y[i];
    double theta = (double)footsteps_theta[i];
    double swing_height = (double)footsteps_swing_height[i];
    double step_duration = (double)footsteps_step_duration[i];
    double step_cost = (double)footsteps_step_cost[i];

    Footstep f(x, y, theta, swing_height, ivEnvironmentParams.lift_height, step_duration, ivEnvironmentParams.sway_duration, step_cost,
               ivEnvironmentParams.cell_size,
               ivEnvironmentParams.num_angle_bins,
               ivEnvironmentParams.hash_table_size);
    ivEnvironmentParams.footstep_set.push_back(f);

    double cur_step_width = sqrt(x*x + y*y);

    if (cur_step_width > ivEnvironmentParams.max_step_dist)
      ivEnvironmentParams.max_step_dist = cur_step_width;
  }



  // load step range polygon
  XmlRpc::XmlRpcValue step_range_x;
  XmlRpc::XmlRpcValue step_range_y;
  nh_private.getParam("step_range/x", step_range_x);
  nh_private.getParam("step_range/y", step_range_y);
  if (!step_range_x.size() || step_range_x.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Error reading step_range/x from config file. Exit!");
    exit(2);
  }
  if (!step_range_y.size() || step_range_y.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Error reading step_range/y from config file. Exit!");
    exit(2);
  }
  if (step_range_x.size() != step_range_y.size())
  {
    ROS_ERROR("Step range points have different size. Exit!");
    exit(2);
  }
  // create step range
  ivEnvironmentParams.step_range.clear();
  ivEnvironmentParams.step_range.reserve(step_range_x.size());
  double max_x = (double)step_range_x[0];
  double max_y = (double)step_range_y[0];
  double max_inv_x = (double)step_range_x[0];
  double max_inv_y = (double)step_range_y[0];
  double cell_size = ivEnvironmentParams.cell_size;
  for (int i=0; i < step_range_x.size(); ++i)
  {
    double x = (double)step_range_x[i];
    double y = (double)step_range_y[i];

    max_x = std::max(max_x, x);
    max_y = std::max(max_y, y);
    max_inv_x = std::min(max_inv_x, x);
    max_inv_y = std::min(max_inv_y, y);

    ivEnvironmentParams.step_range.push_back(
      std::pair<int, int>(disc_val(x, cell_size), disc_val(y, cell_size)));
  }
  // insert first point again at the end!
  ivEnvironmentParams.step_range.push_back(ivEnvironmentParams.step_range[0]);
  ivEnvironmentParams.max_step_range_x = max_x;
  ivEnvironmentParams.max_step_range_y = max_y;
  nh_private.param("foot/max/step/theta", ivEnvironmentParams.max_step_range_theta, 0.3);
  ivEnvironmentParams.max_inverse_step_range_x = max_inv_x;
  ivEnvironmentParams.max_inverse_step_range_y = max_inv_y;
  nh_private.param("foot/max/inverse/step/theta", ivEnvironmentParams.max_inverse_step_range_theta, -0.3);

  max_x = std::max(std::abs(max_x), std::abs(max_inv_x));
  max_y = std::max(std::abs(max_y), std::abs(max_inv_y));
  ivEnvironmentParams.max_step_range_width = sqrt(max_x*max_x + max_y*max_y);

  // initialize the planner environment
  ivPlannerEnvironmentPtr.reset(new FootstepPlannerEnvironment(ivEnvironmentParams));

  // set up planner
  setPlanner();

  publishParams();
}

FootstepPlanner::~FootstepPlanner()
{}


void FootstepPlanner::setPlanner()
{
  if (ivPlannerType == "ARAPlanner" ||
      ivPlannerType == "ADPlanner"  ||
      ivPlannerType == "RSTARPlanner" )
  {
    ROS_INFO_STREAM("Planning with " << ivPlannerType);
  }
  else
  {
    ROS_ERROR_STREAM("Planner "<< ivPlannerType <<" not available / "
                     "untested.");
    exit(1);
  }
  if (ivEnvironmentParams.forward_search)
  {
    ROS_INFO_STREAM("Search direction: forward planning");
  }
  else
  {
    ROS_INFO_STREAM("Search direction: backward planning");
  }

  if (ivPlannerType == "ARAPlanner")
  {
    ivPlannerPtr.reset(
        new ARAPlanner(ivPlannerEnvironmentPtr.get(),
                       ivEnvironmentParams.forward_search));
  }
  else if (ivPlannerType == "ADPlanner")
  {
    ivPlannerPtr.reset(
        new ADPlanner(ivPlannerEnvironmentPtr.get(),
                      ivEnvironmentParams.forward_search));
  }
  else if (ivPlannerType == "RSTARPlanner")
  {
    RSTARPlanner* p =
        new RSTARPlanner(ivPlannerEnvironmentPtr.get(),
                         ivEnvironmentParams.forward_search);
    // new options, require patched SBPL
    //          p->set_local_expand_thres(500);
    //          p->set_eps_step(1.0);
    ivPlannerPtr.reset(p);
  }
  //        else if (ivPlannerType == "ANAPlanner")
  //        	ivPlannerPtr.reset(new anaPlanner(ivPlannerEnvironmentPtr.get(),
  //        	                                  ivForwardSearch));
}


bool FootstepPlanner::run()
{
  bool path_existed = (bool)ivPath.size();
  int ret = 0;
  MDPConfig mdp_config;
  std::vector<int> solution_state_ids;

  // commit start/goal poses to the environment
  /// @TODO: updateGoal adds goal states to list so planner may use it independend from costs...
  ivPlannerEnvironmentPtr->updateStart(ivStartFootLeft, ivStartFootRight);
  ivPlannerEnvironmentPtr->updateGoal(ivGoalFootLeft, ivGoalFootRight);
  ivPlannerEnvironmentPtr->setPlannerStartAndGoal();
  ivPlannerEnvironmentPtr->updateHeuristicValues();
  ivPlannerEnvironmentPtr->InitializeEnv(NULL);
  ivPlannerEnvironmentPtr->InitializeMDPCfg(&mdp_config);

  // inform AD planner about changed (start) states for replanning
  if (path_existed &&
      !ivEnvironmentParams.forward_search &&
      ivPlannerType == "ADPlanner")
  {
    std::vector<int> changed_edges;
    changed_edges.push_back(mdp_config.startstateid);
    // update the AD planner
    boost::shared_ptr<ADPlanner> ad_planner =
      boost::dynamic_pointer_cast<ADPlanner>(ivPlannerPtr);
    ad_planner->update_preds_of_changededges(&changed_edges);
  }

  // set up SBPL
  if (ivPlannerPtr->set_start(mdp_config.startstateid) == 0)
  {
    ROS_ERROR("Failed to set start state.");
    return false;
  }
  if (ivPlannerPtr->set_goal(mdp_config.goalstateid) == 0)
  {
    ROS_ERROR("Failed to set goal state\n");
    return false;
  }

  ivPlannerPtr->set_initialsolution_eps(ivInitialEpsilon);
  ivPlannerPtr->set_search_mode(ivSearchUntilFirstSolution);

  ROS_INFO("Start planning (max time: %f, initial eps: %f (%f), planning mode: %i)\n",
           ivMaxSearchTime, ivInitialEpsilon,
           ivPlannerPtr->get_initial_eps(),
           ivPlannerEnvironmentPtr->getPlanningMode());
  int path_cost;
  ros::WallTime startTime = ros::WallTime::now();
  try
  {
    ret = ivPlannerPtr->replan(ivMaxSearchTime, &solution_state_ids, &path_cost);
  }
  catch (const SBPL_Exception* e)
  {
    ROS_ERROR("SBPL planning failed (%s)", e->what());
    return false;
  }
  ivPathCost = double(path_cost) / FootstepPlannerEnvironment::cvMmScale;

  // cache statistics
  /// TODO
//  if (ivEnvironmentParams.gpr && ivEnvironmentParams.gpr->getCache().getSize() > 0)
//  {
//    ROS_INFO("GPR Cache (size/hit/miss): %lu / %lu / %lu", ivEnvironmentParams.gpr->getCache().getSize(),
//                                                           ivEnvironmentParams.gpr->getCache().getHit(),
//                                                           ivEnvironmentParams.gpr->getCache().getMiss());

//    // save in case of cache update
//    if (ivEnvironmentParams.gpr->getCache().getMiss() > 1000)
//    {
//      ROS_INFO("GPR Cache growed with %lu new entries. Saving...", ivEnvironmentParams.gpr->getCache().getMiss());
//      ivEnvironmentParams.gpr->save(ivEnvironmentParams.gpr_filename);
//      ivEnvironmentParams.gpr->load(ivEnvironmentParams.gpr_filename); // only reload will reset misses
//      ROS_INFO("Done!");
//    }
//  }

  bool path_is_new = pathIsNew(solution_state_ids);
  if (ret && solution_state_ids.size() > 0 && path_is_new)
  {
    ROS_INFO("Solution of size %zu found after %f s",
             solution_state_ids.size(),
             (ros::WallTime::now()-startTime).toSec());

    if (extractPath(solution_state_ids))
    {
      ROS_INFO("Expanded states: %i total / %i new",
               ivPlannerEnvironmentPtr->getNumExpandedStates(),
               ivPlannerPtr->get_n_expands());
      ROS_INFO("Final eps: %f", ivPlannerPtr->get_final_epsilon());
      ROS_INFO("Path cost: %f (%i)\n", ivPathCost, path_cost);

      ivPlanningStatesIds = solution_state_ids;

      broadcastExpandedNodesVis();
      broadcastRandomNodesVis();
      broadcastFootstepPathVis();
      broadcastPathVis();

      return true;
    }
    else
    {
      ROS_ERROR("extracting path failed\n\n");
      return false;
    }
  }
  else if (!path_is_new)
  {
    ROS_ERROR("Solution found by SBPL is the same as the old solution. "
              "Replanning failed.");
    return false;
  }
  else
  {
    broadcastExpandedNodesVis();
    broadcastRandomNodesVis();

    ROS_ERROR("No solution found");
    return false;
  }
}


bool FootstepPlanner::extractPath(const std::vector<int>& state_ids)
{
  ivPath.clear();

  State s;
  State start_left;
  std::vector<int>::const_iterator state_ids_iter = state_ids.begin();

  // first state is always the robot's left foot
  if (!ivPlannerEnvironmentPtr->getState(*state_ids_iter, start_left))
  {
    ivPath.clear();
    return false;
  }
  ++state_ids_iter;
  if (!ivPlannerEnvironmentPtr->getState(*state_ids_iter, s))
  {
    ivPath.clear();
    return false;
  }
  ++state_ids_iter;

  // check if the robot's left foot can be ommited as first state in the path,
  // i.e. the robot's right foot is appended first to the path
  if (s.getLeg() == LEFT)
    ivPath.push_back(ivStartFootRight);
  else
    ivPath.push_back(start_left);
  ivPath.push_back(s);

  for(; state_ids_iter < state_ids.end(); ++state_ids_iter)
  {
    if (!ivPlannerEnvironmentPtr->getState(*state_ids_iter, s))
    {
      ivPath.clear();
      return false;
    }
    ivPath.push_back(s);
  }

  // add last neutral step
  if (ivPath.back().getLeg() == RIGHT)
    ivPath.push_back(ivGoalFootLeft);
  else // last_leg == LEFT
    ivPath.push_back(ivGoalFootRight);

  return true;
}


bool FootstepPlanner::setParamsService(flor_footstep_planner_msgs::FootstepPlannerParamsService::Request &req, flor_footstep_planner_msgs::FootstepPlannerParamsService::Response &/*resp*/)
{
  if (req.params.change_mask)
    ROS_INFO("Received new parameters:");

  // set step cost type
  if (req.params.change_mask & flor_footstep_planner_msgs::FootstepPlannerParams::STEP_COST_ESTIMATOR)
  {
    ROS_INFO("STEP_COST_ESTIMATOR");

    switch(req.params.step_cost_type)
    {
      case EUCLIDEAN_STEP_COST:
      {
        ivEnvironmentParams.step_cost_estimator_type_name = "EuclideanStepCostEstimator";
        ivEnvironmentParams.heuristic_type_name = "EuclStepCostHeuristic";
        break;
      }
      case GPR_STEP_COST:
      {
        ivEnvironmentParams.step_cost_estimator_type_name = "GprStepCostEstimator";
        ivEnvironmentParams.heuristic_type_name = "GPRStepCostHeuristic";
        break;
      }
      case MAP_STEP_COST:
      {
        ivEnvironmentParams.step_cost_estimator_type_name = "MapStepCostEstimator";
        ivEnvironmentParams.heuristic_type_name = "EuclStepCostHeuristic";
        break;
      }
      case BOUNDARY_STEP:
      {
        ivEnvironmentParams.step_cost_estimator_type_name = "BoundaryStepCostEstimator";
        ivEnvironmentParams.heuristic_type_name = "EuclStepCostHeuristic";
        break;
      }
      case DYNAMICS_STEP:
      {
        ivEnvironmentParams.step_cost_estimator_type_name = "DynamicsStepCostEstimator";
        ivEnvironmentParams.heuristic_type_name = "EuclStepCostHeuristic";
        break;
      }
    }
  }

  if (req.params.change_mask & flor_footstep_planner_msgs::FootstepPlannerParams::FOOTSTEP_SET)
  {
    ROS_INFO("FOOTSTEP_SET");

    ivEnvironmentParams.footstep_set.clear();
    ivEnvironmentParams.max_step_dist = 0;

    // load footstep set
    bool use_step_cost = true;
    if (req.params.footstep_set.size() != req.params.footstep_cost.size())
    {
      ROS_WARN("No step costs are given or wrong number of step costs are given in array. Use zero step cost for all steps.");
      use_step_cost = false;
    }

    for (unsigned int i = 0; i < req.params.footstep_set.size(); i++)
    {
      const flor_atlas_msgs::AtlasBehaviorStepData &step = req.params.footstep_set[i];
      double step_cost = use_step_cost ? req.params.footstep_cost[i] : 0.0;

      Footstep f(step.position.x, step.position.y, step.yaw, step.swing_height, ivEnvironmentParams.lift_height, step.duration, ivEnvironmentParams.sway_duration, step_cost,
                 ivEnvironmentParams.cell_size, ivEnvironmentParams.num_angle_bins, ivEnvironmentParams.hash_table_size);
                 ivEnvironmentParams.footstep_set.push_back(f);

      double cur_step_width = sqrt(step.position.x*step.position.x + step.position.y*step.position.y);

      if (cur_step_width > ivEnvironmentParams.max_step_dist)
        ivEnvironmentParams.max_step_dist = cur_step_width;
    }
  }

  if (req.params.change_mask & flor_footstep_planner_msgs::FootstepPlannerParams::LOAD_GPR_STEP_COST)
  {
    ROS_INFO("LOAD_GPR_STEP_COST");
    ivEnvironmentParams.gpr_filename = req.params.gpr_step_cost_file.data.data();
  }

  if (req.params.change_mask & flor_footstep_planner_msgs::FootstepPlannerParams::LOAD_MAP_STEP_COST)
  {
    ROS_INFO("LOAD_MAP_STEP_COST");
    ivEnvironmentParams.map_step_cost_filename = req.params.map_step_cost_file.data.data();
  }

  // set collision mode
  if (req.params.change_mask & flor_footstep_planner_msgs::FootstepPlannerParams::COLLISION_CHECK_TYPE)
  {
    ROS_INFO("COLLISION_CHECK_TYPE");
    ivEnvironmentParams.collision_check_type = req.params.collision_check_type;
  }

  // set foot size
  if (req.params.change_mask & flor_footstep_planner_msgs::FootstepPlannerParams::FOOT_SIZE)
  {
    ROS_INFO("FOOT_SIZE");
    ivEnvironmentParams.foot_size = req.params.foot_size;
    ivEnvironmentParams.foot_origin_shift = req.params.foot_origin_shift;
    ivEnvironmentParams.foot_seperation = req.params.foot_seperation;
  }

  // set upper body dimensions
  if (req.params.change_mask & flor_footstep_planner_msgs::FootstepPlannerParams::UPPER_BODY_SIZE)
  {
    ROS_INFO("UPPER_BODY_SIZE");
    ivEnvironmentParams.upper_body_size = req.params.upper_body_size;
    ivEnvironmentParams.upper_body_origin_shift = req.params.upper_body_origin_shift;
  }

  // standard step params
  if (req.params.change_mask & flor_footstep_planner_msgs::FootstepPlannerParams::STANDARD_STEP_PARAMS)
  {
    ROS_INFO("STANDARD_STEP_PARAMS");
    ivEnvironmentParams.step_duration = req.params.step_duration;
    ivEnvironmentParams.sway_duration = req.params.sway_duration;
    ivEnvironmentParams.swing_height = req.params.swing_height;
    ivEnvironmentParams.lift_height = req.params.lift_height;
  }

  // terrain model
  if (req.params.change_mask & flor_footstep_planner_msgs::FootstepPlannerParams::TERRAIN_MODEL)
  {
    ROS_INFO("TERRAIN_MODEL");
    ivEnvironmentParams.use_terrain_model = req.params.use_terrain_model;
    ivEnvironmentParams.foot_contact_min_sampling_steps_x = req.params.min_sampling_steps_x;
    ivEnvironmentParams.foot_contact_min_sampling_steps_y = req.params.min_sampling_steps_y;
    ivEnvironmentParams.foot_contact_max_sampling_steps_x = req.params.max_sampling_steps_x;
    ivEnvironmentParams.foot_contact_max_sampling_steps_y = req.params.max_sampling_steps_y;
    ivEnvironmentParams.foot_contact_max_intrusion_z = req.params.max_intrusion_z;
    ivEnvironmentParams.foot_contact_max_ground_clearance = req.params.max_ground_clearance;
    ivEnvironmentParams.foot_contact_minimal_support = req.params.minimal_support;
  }

  // reinitialize the planner environment
  if (req.params.change_mask)
  {
    ivPlannerEnvironmentPtr.reset(new FootstepPlannerEnvironment(ivEnvironmentParams));
    if (ivGroundLevelMapPtr)
      ivPlannerEnvironmentPtr->updateGroundLevelMap(ivGroundLevelMapPtr);
    if (ivBodyLevelMapPtr)
      ivPlannerEnvironmentPtr->updateBodyLevelMap(ivBodyLevelMapPtr);
    if (ivTerrainModelPtr)
      ivPlannerEnvironmentPtr->updateTerrainModel(ivTerrainModelPtr);
  }

  publishParams();

  return true;
}


void FootstepPlanner::reset()
{
  // reset the previously calculated paths
  ivPath.clear();
  ivPlanningStatesIds.clear();
  ivCheckedFootContactSupport->clear();
  // reset the planner
  // INFO: force_planning_from_scratch was not working properly the last time
  // checked; therefore instead of using this function the planner is manually
  // reset
  //ivPlannerPtr->force_planning_from_scratch();
  ivPlannerEnvironmentPtr->reset();
  setPlanner();
}


void FootstepPlanner::resetTotally()
{
  // reset the previously calculated paths
  ivPath.clear();
  ivPlanningStatesIds.clear();
  ivCheckedFootContactSupport->clear();
  // reinitialize the planner environment
  ivPlannerEnvironmentPtr.reset(new FootstepPlannerEnvironment(ivEnvironmentParams));
  setPlanner();
}


bool FootstepPlanner::plan(uint32_t &status_flags)
{
  status_flags = flor_footstep_planner_msgs::PlanFootstepsResponse::NONE;

  if ((ivEnvironmentParams.collision_check_type & FootstepPlannerEnvironment::CHECK_UPPER_BODY) && !ivBodyLevelMapPtr)
  {
    ROS_ERROR("FootstepPlanner has no body level map for planning yet.");
    status_flags |= flor_footstep_planner_msgs::PlanFootstepsResponse::NO_BODY_LEVEL_GRID_MAP;
  }

  if ((ivEnvironmentParams.collision_check_type & FootstepPlannerEnvironment::CHECK_FOOT) && !ivGroundLevelMapPtr)
  {
    ROS_ERROR("FootstepPlanner has no ground level grid map for planning yet.");
    status_flags |= flor_footstep_planner_msgs::PlanFootstepsResponse::NO_GROUND_LEVEL_GRID_MAP;
  }

  if ((ivEnvironmentParams.collision_check_type & FootstepPlannerEnvironment::FOOT_SUPPORT_AREA) && !ivPlannerEnvironmentPtr->getTerrainModel())
  {
    ROS_ERROR("FootstepPlanner has no terrain model for planning yet.");
    status_flags |= flor_footstep_planner_msgs::PlanFootstepsResponse::NO_TERRAIN_MODEL;
  }

  if (!ivStartPoseSetUp)
  {
    ROS_ERROR("FootstepPlanner has not set the start pose yet.");
    status_flags |= flor_footstep_planner_msgs::PlanFootstepsResponse::START_POSE_INACCESSIBLE;
  }

  if (!ivGoalPoseSetUp)
  {
    ROS_ERROR("FootstepPlanner has not set the goal pose yet.");
    status_flags |= flor_footstep_planner_msgs::PlanFootstepsResponse::GOAL_POSE_INACCESSIBLE;
  }

  if (status_flags != flor_footstep_planner_msgs::PlanFootstepsResponse::NONE)
    return false;

  reset();
  // start the planning and return success
  return run();
}


bool FootstepPlanner::replan(uint32_t &status_flags)
{
  status_flags = flor_footstep_planner_msgs::PlanFootstepsResponse::NONE;

  if ((ivEnvironmentParams.collision_check_type & FootstepPlannerEnvironment::CHECK_UPPER_BODY) && !ivBodyLevelMapPtr)
  {
    ROS_ERROR("FootstepPlanner has no body level map for planning yet.");
    status_flags |= flor_footstep_planner_msgs::PlanFootstepsResponse::NO_BODY_LEVEL_GRID_MAP;
  }

  if ((ivEnvironmentParams.collision_check_type & FootstepPlannerEnvironment::CHECK_FOOT) && !ivGroundLevelMapPtr)
  {
    ROS_ERROR("FootstepPlanner has no ground level grid map for planning yet.");
    status_flags |= flor_footstep_planner_msgs::PlanFootstepsResponse::NO_GROUND_LEVEL_GRID_MAP;
  }

  if ((ivEnvironmentParams.collision_check_type & FootstepPlannerEnvironment::FOOT_SUPPORT_AREA) && !ivPlannerEnvironmentPtr->getTerrainModel())
  {
    ROS_ERROR("FootstepPlanner has no terrain model for planning yet.");
    status_flags |= flor_footstep_planner_msgs::PlanFootstepsResponse::NO_TERRAIN_MODEL;
  }

  if (!ivStartPoseSetUp)
  {
    ROS_ERROR("FootstepPlanner has not set the start pose yet.");
    status_flags |= flor_footstep_planner_msgs::PlanFootstepsResponse::START_POSE_INACCESSIBLE;
  }

  if (!ivGoalPoseSetUp)
  {
    ROS_ERROR("FootstepPlanner has not set the goal pose yet.");
    status_flags |= flor_footstep_planner_msgs::PlanFootstepsResponse::GOAL_POSE_INACCESSIBLE;
  }

  if (status_flags != flor_footstep_planner_msgs::PlanFootstepsResponse::NONE)
    return false;

  // Workaround for R* and ARA: need to reinit. everything
  if (ivPlannerType == "RSTARPlanner" || ivPlannerType == "ARAPlanner")
  {
    ROS_INFO("Reset planning information because planner cannot handle "
             "replanning.");
    reset();
  }

  return run();
}


bool FootstepPlanner::plan(const geometry_msgs::PoseStampedConstPtr start,
                           const geometry_msgs::PoseStampedConstPtr goal)
{
  return plan(start->pose.position.x, start->pose.position.y, tf::getYaw(start->pose.orientation),
              goal->pose.position.x, goal->pose.position.y, tf::getYaw(goal->pose.orientation));
}


bool FootstepPlanner::plan(float start_x, float start_y, float start_theta,
                      float goal_x, float goal_y, float goal_theta)
{
  if (!(setStart(start_x, start_y, start_theta) && setGoal(goal_x, goal_y, goal_theta)))
    return false;

  uint32_t status_flags = 0;
  return plan(status_flags);
}


bool FootstepPlanner::plan(flor_footstep_planner_msgs::PlanFootsteps::Request &req,
                           flor_footstep_planner_msgs::PlanFootsteps::Response &resp)
{
  // set start foot poses
  setStartFromReq(req, true); /// @TODO: Hack to disable collision check for start pose

  // set goal foot poses
  setGoalFromReq(req);

  return plan(resp.status_flags);
}


bool FootstepPlanner::planStepping(flor_footstep_planner_msgs::PlanStepping::Request &req, flor_footstep_planner_msgs::PlanStepping::Response &resp)
{
  double cell_size = 0.001;
  int num_angle_bins = 512;
  double angle_bin_size = (2.0*M_PI / num_angle_bins);

  if (req.stepping_request.stepping.generate_terrain_model)
  {
    flor_terrain_classifier::TerrainModelService srv;
    srv.request.terrain_model_request.use_default_region_request = true;
    if (ivTerrainModelService.call(srv.request, srv.response))
      terrainModelCallback(flor_terrain_classifier::TerrainModelConstPtr(new flor_terrain_classifier::TerrainModel(srv.response.terrain_model)));
    else
    {
      ROS_ERROR("Can't get terrain data!");
      resp.status_flags |= flor_footstep_planner_msgs::PlanFootstepsResponse::NO_TERRAIN_MODEL;
      return false;
    }
  }
//  else if (!req.stepping_request.stepping.ignore_collision && !ivGroundLevelMapPtr)
//  {
//    ROS_ERROR("Distance map hasn't been initialized yet.");
//    return false;
//  }

  ROS_INFO("Start planning stepping (walk mode: %u, steps: %u, planning mode: %i)\n",
           req.stepping_request.stepping.walk_mode,
           req.stepping_request.stepping.steps,
           ivPlannerEnvironmentPtr->getPlanningMode());

  // set start foot poses
  if (!setStartFromReq(req.stepping_request, req.stepping_request.stepping.ignore_collision))
  {
    resp.status_flags |= flor_footstep_planner_msgs::PlanFootstepsResponse::START_POSE_INACCESSIBLE;
    return false;
  }

  // single footstep primitve (TODO: GPR may be also possible)
  boost::shared_ptr<Footstep> footstep;
  boost::shared_ptr<Footstep> footstep_sec;

  State current_state;
  bool single_step_mode = false;

  double extra_seperation_factor = req.stepping_request.stepping.extra_seperation ? 1.25 : 1.0;

  switch (req.stepping_request.stepping.walk_mode)
  {
    case flor_footstep_planner_msgs::Stepping::FORWARD:
      footstep = boost::shared_ptr<Footstep>(
                 new Footstep(req.stepping_request.stepping.step_distance_forward, ivEnvironmentParams.foot_seperation+req.stepping_request.stepping.step_distance_sideward, req.stepping_request.stepping.turn_angle,
                              ivEnvironmentParams.swing_height,
                              ivEnvironmentParams.lift_height,
                              ivEnvironmentParams.step_duration,
                              ivEnvironmentParams.sway_duration,
                              0.0,
                              cell_size,
                              num_angle_bins,
                              ivEnvironmentParams.hash_table_size));
      current_state = req.stepping_request.stepping.swap_start_foot ? ivStartFootRight : ivStartFootLeft;;
      break;
    case flor_footstep_planner_msgs::Stepping::BACKWARD:
      footstep = boost::shared_ptr<Footstep>(
                 new Footstep(-req.stepping_request.stepping.step_distance_forward, ivEnvironmentParams.foot_seperation+req.stepping_request.stepping.step_distance_sideward, -req.stepping_request.stepping.turn_angle,
                              ivEnvironmentParams.swing_height,
                              ivEnvironmentParams.lift_height,
                              ivEnvironmentParams.step_duration,
                              ivEnvironmentParams.sway_duration,
                              0.0,
                              cell_size,
                              num_angle_bins,
                              ivEnvironmentParams.hash_table_size));
      current_state = req.stepping_request.stepping.swap_start_foot ? ivStartFootRight : ivStartFootLeft;;
      break;
    case flor_footstep_planner_msgs::Stepping::STRAFE_LEFT:
      footstep = boost::shared_ptr<Footstep>(
                 new Footstep(0.0, ivEnvironmentParams.foot_seperation+req.stepping_request.stepping.step_distance_sideward, 0.0,
                              ivEnvironmentParams.swing_height,
                              ivEnvironmentParams.lift_height,
                              ivEnvironmentParams.step_duration,
                              ivEnvironmentParams.sway_duration,
                              0.0,
                              cell_size,
                              num_angle_bins,
                              ivEnvironmentParams.hash_table_size));
      current_state = ivStartFootRight;
      single_step_mode = true;
      break;
    case flor_footstep_planner_msgs::Stepping::STRAFE_RIGHT:
      footstep = boost::shared_ptr<Footstep>(
                 new Footstep(0.0, ivEnvironmentParams.foot_seperation+req.stepping_request.stepping.step_distance_sideward, 0.0,
                              ivEnvironmentParams.swing_height,
                              ivEnvironmentParams.lift_height,
                              ivEnvironmentParams.step_duration,
                              ivEnvironmentParams.sway_duration,
                              0.0,
                              cell_size,
                              num_angle_bins,
                              ivEnvironmentParams.hash_table_size));
      current_state = ivStartFootLeft;
      single_step_mode = true;
      break;
    case flor_footstep_planner_msgs::Stepping::ROTATE_LEFT:
      footstep = boost::shared_ptr<Footstep>(
                 new Footstep(-sin(req.stepping_request.stepping.turn_angle)*(ivEnvironmentParams.foot_seperation*extra_seperation_factor)/2,
                              cos(req.stepping_request.stepping.turn_angle)*(ivEnvironmentParams.foot_seperation*extra_seperation_factor)/2+(ivEnvironmentParams.foot_seperation*extra_seperation_factor)/2,
                              req.stepping_request.stepping.turn_angle,
                              ivEnvironmentParams.swing_height,
                              ivEnvironmentParams.lift_height,
                              ivEnvironmentParams.step_duration,
                              ivEnvironmentParams.sway_duration,
                              0.0,
                              cell_size,
                              num_angle_bins,
                              ivEnvironmentParams.hash_table_size));
      current_state = ivStartFootRight;
      single_step_mode = true;
      break;
    case flor_footstep_planner_msgs::Stepping::ROTATE_RIGHT:
      footstep = boost::shared_ptr<Footstep>(
                 new Footstep(-sin(req.stepping_request.stepping.turn_angle)*(ivEnvironmentParams.foot_seperation*extra_seperation_factor)/2,
                              cos(req.stepping_request.stepping.turn_angle)*(ivEnvironmentParams.foot_seperation*extra_seperation_factor)/2+(ivEnvironmentParams.foot_seperation*extra_seperation_factor)/2,
                              req.stepping_request.stepping.turn_angle,
                              ivEnvironmentParams.swing_height,
                              ivEnvironmentParams.lift_height,
                              ivEnvironmentParams.step_duration,
                              ivEnvironmentParams.sway_duration,
                              0.0,
                              cell_size,
                              num_angle_bins,
                              ivEnvironmentParams.hash_table_size));
      current_state = ivStartFootLeft;
      single_step_mode = true;
      break;
    case flor_footstep_planner_msgs::Stepping::SAMPLING:
//      footstep = &ivEnvironmentParams.footstep_stepping_set[0];
//      footstep_sec = &ivEnvironmentParams.footstep_stepping_set[1];
//      current_state = ivStartFootLeft;
      break;
    default:
      ROS_ERROR("PlanStepping: Unknown walk mode set!");
      return false;
  }

  // generate simple path via pattern generator
  /// @TODO: check for collision
  if (!addStepping(req, current_state, false))
    return false;

  for (unsigned int i = 0; i < req.stepping_request.stepping.steps; i++)
  {
    PlanningState next = footstep->performMeOnThisState(PlanningState(current_state, cell_size, angle_bin_size, ivEnvironmentParams.hash_table_size),
                                                        req.stepping_request.stepping.use_terrain ? ivPlannerEnvironmentPtr->getTerrainModel() : TerrainModel::ConstPtr());
    current_state = next.getState();
    if (!addStepping(req, current_state))
      return false;

    // if sampling mode, use secondary footstep (should be dual to first)
    if (req.stepping_request.stepping.walk_mode == flor_footstep_planner_msgs::Stepping::SAMPLING)
    {
      next = footstep_sec->performMeOnThisState(PlanningState(current_state, cell_size, angle_bin_size, ivEnvironmentParams.hash_table_size),
                                                req.stepping_request.stepping.use_terrain ? ivPlannerEnvironmentPtr->getTerrainModel() : TerrainModel::ConstPtr());
      current_state = next.getState();
      if (!addStepping(req, current_state))
        return false;
      i++;
    }
    // in single step mode, the second foot should be placed parallel after each step
    else if (single_step_mode && i < (unsigned int)(req.stepping_request.stepping.steps-1))
    {
      current_state = getParallelFootPose(current_state, req.stepping_request.stepping.use_terrain, ivEnvironmentParams.foot_seperation*(extra_seperation_factor-1.0));
      if (!addStepping(req, current_state))
        return false;
    }
  }

  // add final step so feet are parallel
  if (req.stepping_request.stepping.close_step && req.stepping_request.stepping.steps > 0)
  {
    current_state = getParallelFootPose(current_state, req.stepping_request.stepping.use_terrain);
    if (!addStepping(req, current_state, false))
      return false;
  }

  return true;
}


bool FootstepPlanner::addStepping(const flor_footstep_planner_msgs::PlanStepping::Request &req, State &s, bool change_z)
{
  if (req.stepping_request.stepping.override)
  {
    if (change_z)
      s.setZ(s.getZ() + req.stepping_request.stepping.goal_z);
    s.setRoll(req.stepping_request.stepping.roll);
    s.setPitch(req.stepping_request.stepping.pitch);
    s.setSwingHeight(req.stepping_request.stepping.swing_height);
    s.setLiftHeight(req.stepping_request.stepping.lift_height);
    s.setKneeNominal(req.stepping_request.stepping.knee_nominal);
    ivPath.push_back(s);
    return true;
  }
  else
  {
    ivPath.push_back(s);
    return true;
  }

  return false;
}


bool FootstepPlanner::planRealignFeet(const flor_footstep_planner_msgs::PlanStepping::Request &req)
{
  ROS_INFO("Start planning feet realignment (mode: %u, using planning mode: %u)\n",
           req.stepping_request.stepping.planning_mode,
           ivPlannerEnvironmentPtr->getPlanningMode());

  // set start foot poses
  setStartFromReq(req.stepping_request, true);

  // set start standing foot
  State current_state;
  switch (req.stepping_request.stepping.planning_mode)
  {
    case flor_footstep_planner_msgs::FootstepPlan::MODE_FEET_REALIGN_ON_CENTER:
    case flor_footstep_planner_msgs::FootstepPlan::MODE_FEET_REALIGN_ON_LEFT:
    case flor_footstep_planner_msgs::FootstepPlan::MODE_WIDE_STANCE:
      current_state = ivStartFootLeft;
      break;
    case flor_footstep_planner_msgs::FootstepPlan::MODE_FEET_REALIGN_ON_RIGHT:
      current_state = ivStartFootRight;
      break;
    default:
      ROS_ERROR("planRealignFeet was given unknown planning mode %u", req.stepping_request.stepping.planning_mode);
      return false;
  }

  // generate mini plan
  ivPath.push_back(current_state);

  switch (req.stepping_request.stepping.planning_mode)
  {
    case flor_footstep_planner_msgs::FootstepPlan::MODE_FEET_REALIGN_ON_CENTER:
    {
      State goal(req.stepping_request.start,
                 ivEnvironmentParams.swing_height,
                 ivEnvironmentParams.lift_height,
                 ivEnvironmentParams.step_duration,
                 ivEnvironmentParams.sway_duration,
                 NOLEG);
      goal.setZ(current_state.getZ());
      ivPath.push_back(getFootPose(goal, RIGHT, req.stepping_request.stepping.use_terrain));
      ivPath.push_back(getFootPose(goal, LEFT, req.stepping_request.stepping.use_terrain));
      break;
    }

    case flor_footstep_planner_msgs::FootstepPlan::MODE_FEET_REALIGN_ON_LEFT:
    case flor_footstep_planner_msgs::FootstepPlan::MODE_FEET_REALIGN_ON_RIGHT:
    {
      current_state = getParallelFootPose(current_state, req.stepping_request.stepping.use_terrain);
      ivPath.push_back(current_state);
      break;
    }

    case flor_footstep_planner_msgs::FootstepPlan::MODE_WIDE_STANCE:
    {
      State goal(req.stepping_request.start,
                 ivEnvironmentParams.swing_height,
                 ivEnvironmentParams.lift_height,
                 ivEnvironmentParams.step_duration,
                 ivEnvironmentParams.sway_duration,
                 NOLEG);
      goal.setZ(current_state.getZ());
      ivPath.push_back(getFootPose(goal, RIGHT, req.stepping_request.stepping.step_distance_forward, req.stepping_request.stepping.step_distance_sideward, req.stepping_request.stepping.turn_angle, req.stepping_request.stepping.use_terrain));
      ivPath.push_back(getFootPose(goal, LEFT, req.stepping_request.stepping.step_distance_forward, req.stepping_request.stepping.step_distance_sideward, req.stepping_request.stepping.turn_angle, req.stepping_request.stepping.use_terrain));
      break;
    }

    default:
      ROS_ERROR("planRealignFeet was given unknown planning mode %u", req.stepping_request.stepping.planning_mode);
      return false;
    }

  return true;
}


bool FootstepPlanner::planFootstepService(flor_footstep_planner_msgs::PlanFootsteps::Request &req,
                                          flor_footstep_planner_msgs::PlanFootsteps::Response &resp)
{
  switch (req.planning_mode)
  {
    case flor_footstep_planner_msgs::FootstepPlan::MODE_WALK:
    case flor_footstep_planner_msgs::FootstepPlan::MODE_STEP:
      ivPlannerEnvironmentPtr->setPlanningMode(req.planning_mode);
      resp.result = plan(req, resp);
      break;
    default:
      ROS_ERROR("planFootstepService was given not supported planning mode %u", req.planning_mode);
      break;
  }

  if (resp.result)
    planService(req, resp);

  resp.costs = getPathCosts();
  resp.final_eps = ivPlannerPtr->get_final_epsilon();
  resp.planning_time = ivPlannerPtr->get_final_eps_planning_time();
  resp.expanded_states = ivPlannerEnvironmentPtr->getNumExpandedStates();

  if (resp.result && resp.final_eps > 1.4)
    resp.status_flags |= flor_footstep_planner_msgs::PlanFootstepsResponse::PLAN_SUBOPTIMAL;

  // return true since service call was successful (independent from the
  // success of the planning call)
  return true;
}


bool FootstepPlanner::planSteppingService(flor_footstep_planner_msgs::PlanStepping::Request &req,
                                          flor_footstep_planner_msgs::PlanStepping::Response &resp)
{
  reset();

  switch (req.stepping_request.stepping.planning_mode)
  {
    case flor_footstep_planner_msgs::FootstepPlan::MODE_FEET_REALIGN_ON_CENTER:
    case flor_footstep_planner_msgs::FootstepPlan::MODE_FEET_REALIGN_ON_LEFT:
    case flor_footstep_planner_msgs::FootstepPlan::MODE_FEET_REALIGN_ON_RIGHT:
    case flor_footstep_planner_msgs::FootstepPlan::MODE_WIDE_STANCE:
      ivPlannerEnvironmentPtr->setPlanningMode(flor_footstep_planner_msgs::FootstepPlan::MODE_STEP);
      resp.result = planRealignFeet(req);
      break;
    case flor_footstep_planner_msgs::FootstepPlan::MODE_WALK:
    case flor_footstep_planner_msgs::FootstepPlan::MODE_STEP:
      ivPlannerEnvironmentPtr->setPlanningMode(req.stepping_request.stepping.planning_mode);
      resp.result = planStepping(req, resp);
      break;
    default:
      ROS_ERROR("planSteppingService was given unknown planning mode %u", req.stepping_request.stepping.planning_mode);
      break;
  }

  if (resp.result)
    planService(req, resp);

  //resp.costs = getPathCosts();
  //resp.final_eps = ivPlannerPtr->get_final_epsilon();
  resp.planning_time = 0.0;
  //resp.expanded_states = ivPlannerEnvironmentPtr->getNumExpandedStates();

  // return true since service call was successful (independent from the
  // success of the planning call)
  return true;
}

void FootstepPlanner::postProcessStep(const State &left_foot, const State &right_foot, flor_footstep_planner_msgs::StepTarget &swing_foot) const
{
  /// set default parameters
  swing_foot.toe_off = flor_atlas_msgs::AtlasBehaviorStepAction::TOE_OFF_ENABLE;
  swing_foot.max_body_accel = 0.0;
  swing_foot.max_foot_vel = 0.0;
  swing_foot.sway_end_dist = 0.1;
  swing_foot.step_end_dist = 0.1;

  const State &swing_foot_before = swing_foot.foot_index == flor_atlas_msgs::AtlasBehaviorFootData::FOOT_LEFT ? left_foot : right_foot;
  const State &stand_foot = swing_foot.foot_index == flor_atlas_msgs::AtlasBehaviorFootData::FOOT_LEFT ? right_foot : left_foot;

  /// handle lift and swing height
  double swing_diff_x = swing_foot.foot.position.x - swing_foot_before.getX();
  double swing_diff_y = swing_foot.foot.position.y - swing_foot_before.getY();

  const TerrainModel::Ptr &terrain_model = ivPlannerEnvironmentPtr->getTerrainModel();
  if (terrain_model)
  {
    // setup sampling along foot trajectory
    double swing_dist = std::sqrt(swing_diff_x*swing_diff_x + swing_diff_y*swing_diff_y);

    double scale = terrain_model->getResolution()/swing_dist;

    unsigned int number_steps = std::floor(1.0/scale);

    double x = swing_foot_before.getX();
    double y = swing_foot_before.getY();

    double step_x = swing_diff_x * scale;
    double step_y = swing_diff_y * scale;

    double max_swing_foot_z = std::max(swing_foot.foot.position.z, swing_foot_before.getZ());
    double min_stance_z = std::min(swing_foot.foot.position.z, stand_foot.getZ());
    double max_stance_z = std::max(swing_foot.foot.position.z, stand_foot.getZ());
    double max_z = max_swing_foot_z;

    // TODO: add max lift height as parameter
    double max_lift_height = std::max(ivEnvironmentParams.lift_height, 0.30 - (max_stance_z-min_stance_z));

    // get max terrain height along foot trajectorie
    for (unsigned int i = 0; i < number_steps; i++, x += step_x, y += step_y)
    {
      double height = max_z;
      if (terrain_model->getHeight(x, y, height))
        max_z = std::max(max_z, height);
    }

    // determine lift height
    if (max_z > max_swing_foot_z) // check if we must step over
      swing_foot.lift_height += max_z-max_swing_foot_z;

    // clamp lift height
    if (swing_foot.lift_height > max_lift_height)
    {
      ROS_WARN("Max lift height exceeded! Got %f; clamp to %f", swing_foot.lift_height, max_lift_height);
      swing_foot.lift_height = max_lift_height;
    }
  }

  // in walk mode add lift to swing height
  if (ivPlannerEnvironmentPtr->getPlanningMode() == flor_footstep_planner_msgs::FootstepPlan::MODE_WALK)
  {
    swing_foot.swing_height += swing_foot.lift_height;
    swing_foot.lift_height = 0.0;
  }

  /// handle step and sway duration
  double swing_rel_stand_foot_dist = std::abs(cos(-stand_foot.getYaw()) * swing_diff_x - sin(-stand_foot.getYaw()) * swing_diff_y);

  if (swing_rel_stand_foot_dist > ivEnvironmentParams.post_processing_sw_max_swing_dist && swing_foot.foot.normal.z < ivEnvironmentParams.post_processing_sw_min_normal_z)
    swing_foot.sway_duration = ivEnvironmentParams.post_processing_sw_adjusted;

  /// handle TOE-OFF
  if (std::abs(angles::shortest_angular_distance(stand_foot.getYaw(), swing_foot.foot.yaw)) > ivEnvironmentParams.post_processing_toe_max_turn_rate)
    swing_foot.toe_off = flor_atlas_msgs::AtlasBehaviorStepAction::TOE_OFF_DISABLE;

  /// handle knee nominal
  if (stand_foot.getZ() - swing_foot.foot.position.z > ivEnvironmentParams.post_processing_kn_max_step_down)
    swing_foot.knee_nominal = ivEnvironmentParams.post_processing_kn_adjusted;
}

bool FootstepPlanner::findNearestValidState(State &s) const
{
  State current_state = s;
  State best_state = s;

  double pos_diff = FLT_MAX;
  double yaw_diff = FLT_MAX;
  bool solution_found = false;

  // get transformation foot -> world
  tf::Transform t;
  t.setOrigin(s.getPose().getOrigin());
  t.setBasis(s.getPose().getBasis());

  tf::Vector3 orig_pos;
  tf::Vector3 trans_pos;
  orig_pos.setZ(0.0);

  for (double yaw = -0.2; yaw <= 0.4; yaw+=ivEnvironmentParams.angle_bin_size)
  {
    current_state.setYaw(s.getYaw() + (s.getLeg() == LEFT ? yaw : -yaw));
    for (double y = -0.05; y <= 0.2; y+=ivEnvironmentParams.cell_size)
    {
      orig_pos.setY(s.getLeg() == LEFT ? y : -y);
      for (double x = -0.15; x <= 0.15; x+=ivEnvironmentParams.cell_size)
      {
        // determine point in world frame and get height at this point
        orig_pos.setX(x);
        trans_pos = t * orig_pos;

        current_state.setX(trans_pos.getX());
        current_state.setY(trans_pos.getY());

        if (ivEnvironmentParams.use_terrain_model && ivPlannerEnvironmentPtr->getTerrainModel())
          ivPlannerEnvironmentPtr->getTerrainModel()->add3DData(current_state);

        if (ivEnvironmentParams.collision_check_type & FootstepPlannerEnvironment::FOOT_SUPPORT_AREA && current_state.getGroundContactSupport() < ivEnvironmentParams.foot_contact_minimal_support)
          continue;
        if (ivEnvironmentParams.collision_check_type & FootstepPlannerEnvironment::CHECK_FOOT && ivPlannerEnvironmentPtr->occupiedFoot(current_state))
          continue;

        double dist = std::sqrt(x*x + y*y);
        if (pos_diff >= dist && yaw_diff >= std::abs(yaw))
        {
          best_state = current_state;
          pos_diff = dist;
          yaw_diff = std::abs(yaw);
          solution_found = true;
        }
      }
    }
  }

  if (solution_found)
    s = best_state;

  return solution_found;
}

bool FootstepPlanner::checkRobotCollision(const State &left_foot, const State &right_foot, bool &left, bool &right) const
{
  left = false;
  right = false;

  if (ivEnvironmentParams.collision_check_type & FootstepPlannerEnvironment::CHECK_UPPER_BODY && ivPlannerEnvironmentPtr->occupiedUpperBody(left_foot, right_foot))
  {
    left = true;
    right = true;
    return true;
  }
  if (ivEnvironmentParams.collision_check_type & FootstepPlannerEnvironment::FOOT_SUPPORT_AREA)
  {
    left = left || left_foot.getGroundContactSupport() < ivEnvironmentParams.foot_contact_minimal_support;
    right = right || right_foot.getGroundContactSupport() < ivEnvironmentParams.foot_contact_minimal_support;
  }
  if (ivEnvironmentParams.collision_check_type & FootstepPlannerEnvironment::CHECK_FOOT)
  {
    left = left || ivPlannerEnvironmentPtr->occupiedFoot(left_foot);
    right = right || ivPlannerEnvironmentPtr->occupiedFoot(right_foot);
  }

  return left || right;
}

void FootstepPlanner::goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& goal_pose)
{
  // update the goal states in the environment
  if (setGoal(goal_pose))
  {
    if (ivStartPoseSetUp)
    {
      uint32_t status_flags = 0;
      // this check enforces a planning from scratch if necessary (dependent on
      // planning direction)
      if (ivEnvironmentParams.forward_search)
        replan(status_flags);
      else
        plan(status_flags);
    }
  }
}


void FootstepPlanner::startPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& start_pose)
{
  if (setStart(start_pose->pose.pose.position.x,
               start_pose->pose.pose.position.y,
               tf::getYaw(start_pose->pose.pose.orientation)))
  {
    if (ivGoalPoseSetUp)
    {
      // this check enforces a planning from scratch if necessary (dependent on
      // planning direction)
      //if (ivEnvironmentParams.forward_search)
      //  plan();
      //else
      //  replan();
    }
  }
}


void FootstepPlanner::groundLevelMapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_map)
{
  frame_id = occupancy_map->header.frame_id;
  GridMap2DPtr map(new GridMap2D(occupancy_map));

  // new map: update the map information
  if (updateGroundLevelMap(map))
  {
    // NOTE: update map currently simply resets the planner, i.e. replanning
    // here is in fact a planning from the scratch
    //replan();
  }
}

void FootstepPlanner::bodyLevelMapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_map)
{
  if (!ivGroundLevelMapPtr)
    frame_id = occupancy_map->header.frame_id;
  GridMap2DPtr map(new GridMap2D(occupancy_map));

  // new map: update the map information
  if (updateBodyLevelMap(map))
  {
    // NOTE: update map currently simply resets the planner, i.e. replanning
    // here is in fact a planning from the scratch
    //replan();
  }
}


void FootstepPlanner::terrainModelCallback(const flor_terrain_classifier::TerrainModelConstPtr& terrain_model)
{
  ivTerrainModelPtr = terrain_model;
  groundLevelMapCallback(nav_msgs::OccupancyGridConstPtr(new nav_msgs::OccupancyGrid(terrain_model->ground_level_map)));
  ivPlannerEnvironmentPtr->updateTerrainModel(terrain_model);
}


bool FootstepPlanner::setGoal(const geometry_msgs::PoseStampedConstPtr goal_pose, bool ignore_collision)
{
  return setGoal(goal_pose->pose.position.x,
                 goal_pose->pose.position.y,
                 tf::getYaw(goal_pose->pose.orientation),
                 ignore_collision);
}

bool FootstepPlanner::setGoal(float x, float y, float yaw, bool ignore_collision)
{
  State goal(x, y, 0.0, 0.0, 0.0, yaw,
             ivEnvironmentParams.swing_height,
             ivEnvironmentParams.lift_height,
             ivEnvironmentParams.step_duration,
             ivEnvironmentParams.sway_duration,
             NOLEG);

  State foot_left = getFootPose(goal, LEFT, ivEnvironmentParams.use_terrain_model);
  State foot_right = getFootPose(goal, RIGHT, ivEnvironmentParams.use_terrain_model);

  bool success = setGoal(foot_left, foot_right, ignore_collision);
  if (!success)
    ROS_ERROR("Goal pose not accessible.");

  return success;
}

bool FootstepPlanner::setGoal(const State& left_foot, const State& right_foot, bool ignore_collision)
{
  ivGoalPoseSetUp = false;
  bool left_collision = false;
  bool right_collision = false;

  ivGoalFootLeft = left_foot;
  ivGoalFootRight = right_foot;

  if (!ignore_collision && checkRobotCollision(ivGoalFootLeft, ivGoalFootRight, left_collision, right_collision))
  {
    if (!ivShiftGoalPose)
      return false;

    if (left_collision)
    {
       if (!findNearestValidState(ivGoalFootLeft))
         return false;
       else if (right_collision)
         ivGoalFootRight = getParallelFootPose(ivGoalFootLeft, ivEnvironmentParams.use_terrain_model);
    }
    if (right_collision && !findNearestValidState(ivGoalFootRight))
      return false;

    // finally recheck upper body collision
    if (left_collision || right_collision)
    {
      if (ivEnvironmentParams.collision_check_type & FootstepPlannerEnvironment::CHECK_UPPER_BODY && ivPlannerEnvironmentPtr->occupiedUpperBody(ivGoalFootLeft, ivGoalFootRight))
        return false;
      ROS_WARN("Goal pose was shifted!");
    }
  }

  ivGoalPoseSetUp = true;

  ROS_INFO("Goal foot poses set to (left: %f %f %f %f) and (right: %f %f %f %f)",
           ivGoalFootLeft.getX(), ivGoalFootLeft.getY(), ivGoalFootLeft.getZ(), ivGoalFootLeft.getYaw(),
           ivGoalFootRight.getX(), ivGoalFootRight.getY(), ivGoalFootRight.getZ(), ivGoalFootRight.getYaw());

  return true;
}


bool FootstepPlanner::setStart(const geometry_msgs::PoseStampedConstPtr start_pose, bool ignore_collision)
{
  return setStart(start_pose->pose.position.x,
                  start_pose->pose.position.y,
                  tf::getYaw(start_pose->pose.orientation),
                  ignore_collision);
}

bool FootstepPlanner::setStart(float x, float y, float yaw, bool ignore_collision)
{
  State start(x, y, 0.0, 0.0, 0.0, yaw,
              ivEnvironmentParams.swing_height,
              ivEnvironmentParams.lift_height,
              ivEnvironmentParams.step_duration,
              ivEnvironmentParams.sway_duration,
              NOLEG);

  State foot_left = getFootPose(start, LEFT, ivEnvironmentParams.use_terrain_model);
  State foot_right = getFootPose(start, RIGHT, ivEnvironmentParams.use_terrain_model);

  bool success = setStart(foot_left, foot_right, ignore_collision);
  if (!success)
    ROS_ERROR("Start pose not accessible.");

  // publish visualization:
  if (ivStartPoseVisPub.getNumSubscribers() > 0)
  {
    if (ivEnvironmentParams.use_terrain_model && ivPlannerEnvironmentPtr->getTerrainModel())
      ivPlannerEnvironmentPtr->getTerrainModel()->add3DData(start);

    geometry_msgs::PoseStamped start_pose;
    start_pose.pose.position.x = start.getX();
    start_pose.pose.position.y = start.getY();
    start_pose.pose.position.z = start.getZ();

    geometry_msgs::Vector3 normal;
    normal.x = start.getNormalX();
    normal.y = start.getNormalY();
    normal.z = start.getNormalZ();
    flor_footstep_planner_msgs::normalToQuaternion(start.getYaw(), normal, start_pose.pose.orientation);

    start_pose.header.frame_id = frame_id;
    start_pose.header.stamp = ros::Time::now();

    ivStartPoseVisPub.publish(start_pose);
  }

  return success;
}

bool FootstepPlanner::setStart(const State& left_foot, const State& right_foot, bool ignore_collision)
{
  ivStartPoseSetUp = true;
  bool left_collision = false;
  bool right_collision = false;

  if (!ignore_collision && checkRobotCollision(left_foot, right_foot, left_collision, right_collision))
  {
    ivStartPoseSetUp = false;
    return false;
  }

  ivStartFootLeft = left_foot;
  ivStartFootRight = right_foot;

  ROS_INFO("Start foot poses set to (left: %f %f %f %f) and (right: %f %f %f %f)",
           ivStartFootLeft.getX(), ivStartFootLeft.getY(), ivStartFootLeft.getZ(), ivStartFootLeft.getYaw(),
           ivStartFootRight.getX(), ivStartFootRight.getY(), ivStartFootRight.getZ(), ivStartFootRight.getYaw());

  return true;
}


/// @TODO: further cleanup needed here
bool FootstepPlanner::updateGroundLevelMap(const GridMap2DPtr& map)
{
  // store old map pointer locally
  GridMap2DPtr old_map = ivGroundLevelMapPtr;
  // store new map
  ivGroundLevelMapPtr = map;

//  // check if a previous map and a path existed
//  if (old_map && (bool)ivPath.size())
//  {
//    //updateEnvironment(old_map);
//    return true;
//  }

  // ..otherwise the environment's map can simply be updated
  ivPlannerEnvironmentPtr->updateGroundLevelMap(map);
  return false;
}


/// @TODO: further cleanup needed here
bool FootstepPlanner::updateBodyLevelMap(const GridMap2DPtr& map)
{
  // store old map locally
  GridMap2DPtr old_map = ivBodyLevelMapPtr;
  // store new map
  ivBodyLevelMapPtr = map;

//  // check if a previous map and a path existed
//  if (old_map && (bool)ivPath.size())
//  {
//    /// @TODO
//    //updateEnvironment(old_map);
//    return true;
//  }

  // ..otherwise the environment's map can simply be updated
  ivPlannerEnvironmentPtr->updateBodyLevelMap(map);
  return false;
}


void FootstepPlanner::updateEnvironment(const GridMap2DPtr old_map)
{
  ROS_INFO("Reseting the planning environment.");
  // reset environment
  resetTotally();
  // set the new map
  ivPlannerEnvironmentPtr->updateGroundLevelMap(ivGroundLevelMapPtr);
}


State FootstepPlanner::getFootPose(const State& robot, Leg leg, bool use_terrain_model)
{
  double shift_x = -sin(robot.getYaw()) * (0.5 * ivEnvironmentParams.foot_seperation);
  double shift_y =  cos(robot.getYaw()) * (0.5 * ivEnvironmentParams.foot_seperation);

  double sign = -1.0;
  if (leg == LEFT)
    sign = 1.0;

  State foot(robot.getX() + sign * shift_x,
             robot.getY() + sign * shift_y,
             robot.getZ(),
             robot.getRoll(),
             robot.getPitch(),
             robot.getYaw(),
             robot.getSwingHeight(),
             robot.getLiftHeight(),
             robot.getStepDuration(),
             robot.getSwayDuration(),
             leg);

  if (use_terrain_model && ivPlannerEnvironmentPtr->getTerrainModel())
    ivPlannerEnvironmentPtr->getTerrainModel()->add3DData(foot);

  return foot;
}

State FootstepPlanner::getFootPose(const State& robot, Leg leg, double dx, double dy, double dyaw, bool use_terrain_model)
{
  double sign = -1.0;
  if (leg == LEFT)
    sign = 1.0;

  double cos_theta = cos(robot.getYaw());
  double sin_theta = sin(robot.getYaw());
  double shift_x = cos_theta * sign * dx - sin_theta * (0.5 * ivEnvironmentParams.foot_seperation + dy);
  double shift_y = sin_theta * sign * dx + cos_theta * (0.5 * ivEnvironmentParams.foot_seperation + dy);

  State foot(robot.getX() + sign * shift_x,
             robot.getY() + sign * shift_y,
             robot.getZ(),
             robot.getRoll(),
             robot.getPitch(),
             robot.getYaw() + sign * dyaw,
             robot.getSwingHeight(),
             robot.getLiftHeight(),
             robot.getStepDuration(),
             robot.getSwayDuration(),
             leg);

  if (use_terrain_model && ivPlannerEnvironmentPtr->getTerrainModel())
    ivPlannerEnvironmentPtr->getTerrainModel()->add3DData(foot);

  return foot;
}

State FootstepPlanner::getParallelFootPose(const State& foot, bool use_terrain_model, double additional_seperation)
{
  double shift_x = -sin(foot.getYaw()) * (ivEnvironmentParams.foot_seperation + additional_seperation);
  double shift_y =  cos(foot.getYaw()) * (ivEnvironmentParams.foot_seperation + additional_seperation);

  double sign = -1.0;
  if (foot.getLeg() == RIGHT)
    sign = 1.0;

  State foot_parallel(foot.getX() + sign * shift_x,
             foot.getY() + sign * shift_y,
             foot.getZ(),
             foot.getRoll(),
             foot.getPitch(),
             foot.getYaw(),
             foot.getSwingHeight(),
             foot.getLiftHeight(),
             foot.getStepDuration(),
             foot.getSwayDuration(),
             foot.getLeg() == RIGHT ? LEFT : RIGHT);

  if (use_terrain_model && ivPlannerEnvironmentPtr->getTerrainModel())
    ivPlannerEnvironmentPtr->getTerrainModel()->add3DData(foot_parallel);

  return foot_parallel;
}

bool
FootstepPlanner::pathIsNew(const std::vector<int>& new_path)
{
  if (new_path.size() != ivPlanningStatesIds.size())
    return true;

  bool unequal = true;
  for (unsigned i = 0; i < new_path.size(); ++i)
    unequal = new_path[i] != ivPlanningStatesIds[i] && unequal;

  return unequal;
}


void FootstepPlanner::clearFootstepPathVis(unsigned num_footsteps)
{
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray marker_msg;

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = frame_id;


  if (num_footsteps < 1)
    num_footsteps = ivLastMarkerMsgSize;

  for (unsigned i = 0; i < num_footsteps; ++i)
  {
    marker.ns = ivMarkerNamespace;
    marker.id = i;
    marker.action = visualization_msgs::Marker::DELETE;

    marker_msg.markers.push_back(marker);
  }

  ivFootstepPathVisPub.publish(marker_msg);
}


void FootstepPlanner::broadcastExpandedNodesVis()
{
  if (ivExpandedStatesVisPub.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud cloud_msg;
    geometry_msgs::Point32 point;
    std::vector<geometry_msgs::Point32> points;

    State s;
    FootstepPlannerEnvironment::exp_states_2d_iter_t state_id_it;
    for(state_id_it = ivPlannerEnvironmentPtr->getExpandedStatesStart();
        state_id_it != ivPlannerEnvironmentPtr->getExpandedStatesEnd();
        ++state_id_it)
    {
      point.x = cell_2_state(state_id_it->first,
                             ivEnvironmentParams.cell_size);
      point.y = cell_2_state(state_id_it->second,
                             ivEnvironmentParams.cell_size);
      point.z = 0.01;
      points.push_back(point);
    }
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = frame_id;

    cloud_msg.points = points;

    //ROS_INFO("Expanded states: %lu", points.size());

    ivExpandedStatesVisPub.publish(cloud_msg);
  }
}


void FootstepPlanner::broadcastFootstepPathVis()
{
  if (getPathSize() == 0)
  {
    ROS_INFO("no path has been extracted yet");
    return;
  }

  clearFootstepPathVis(0);

  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray broadcast_msg;
  std::vector<visualization_msgs::Marker> markers;

  int markers_counter = 0;

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = frame_id;

  // add the missing start foot to the publish vector for visualization:
  if (ivPath.front().getLeg() == LEFT)
    footPoseToMarker(ivStartFootRight, &marker);
  else
    footPoseToMarker(ivStartFootLeft, &marker);
  marker.id = markers_counter++;
  markers.push_back(marker);

  // add the footsteps of the path to the publish vector
  for(state_iter_t path_iter = getPathBegin(); path_iter != getPathEnd(); ++path_iter)
  {
    footPoseToMarker(*path_iter, &marker);
    marker.id = markers_counter++;
    markers.push_back(marker);
  }

  broadcast_msg.markers = markers;
  ivLastMarkerMsgSize = markers.size();

  ivFootstepPathVisPub.publish(broadcast_msg);
}


void FootstepPlanner::broadcastRandomNodesVis()
{
  if (ivRandomStatesVisPub.getNumSubscribers() > 0){
    sensor_msgs::PointCloud cloud_msg;
    geometry_msgs::Point32 point;
    std::vector<geometry_msgs::Point32> points;

    State s;
    FootstepPlannerEnvironment::exp_states_iter_t state_id_iter;
    for(state_id_iter = ivPlannerEnvironmentPtr->getRandomStatesStart();
        state_id_iter != ivPlannerEnvironmentPtr->getRandomStatesEnd();
        ++state_id_iter)
    {
      if (!ivPlannerEnvironmentPtr->getState(*state_id_iter, s))
      {
        ROS_WARN("Could not get random state %d", *state_id_iter);
      }
      else
      {
        point.x = s.getX();
        point.y = s.getY();
        point.z = s.getZ();
        points.push_back(point);
      }
    }
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = frame_id;

    cloud_msg.points = points;

    ivRandomStatesVisPub.publish(cloud_msg);
  }
}


void FootstepPlanner::broadcastPathVis()
{
  if (getPathSize() == 0)
  {
    ROS_INFO("no path has been extracted yet");
    return;
  }

  nav_msgs::Path path_msg;
  geometry_msgs::PoseStamped state;

  state.header.stamp = ros::Time::now();
  state.header.frame_id = frame_id;

  state_iter_t path_iter;
  for(path_iter = getPathBegin(); path_iter != getPathEnd(); ++path_iter)
  {
    state.pose.position.x = path_iter->getX();
    state.pose.position.y = path_iter->getY();
    state.pose.position.z = path_iter->getZ();
    path_msg.poses.push_back(state);
  }

  path_msg.header = state.header;
  ivPathVisPub.publish(path_msg);
}

void FootstepPlanner::footPoseToMarker(const State& foot_pose, visualization_msgs::Marker* marker)
{
  marker->header.stamp = ros::Time::now();
  marker->header.frame_id = frame_id;
  marker->ns = ivMarkerNamespace;
  marker->type = visualization_msgs::Marker::CUBE;
  marker->action = visualization_msgs::Marker::ADD;

  float cos_theta = cos(foot_pose.getYaw());
  float sin_theta = sin(foot_pose.getYaw());
  float x_shift = cos_theta * ivEnvironmentParams.foot_origin_shift.x -
                  sin_theta * ivEnvironmentParams.foot_origin_shift.y;
  float y_shift;
  if (foot_pose.getLeg() == LEFT)
    y_shift = sin_theta * ivEnvironmentParams.foot_origin_shift.x +
              cos_theta * ivEnvironmentParams.foot_origin_shift.y;
  else // leg == RLEG
    y_shift = sin_theta * ivEnvironmentParams.foot_origin_shift.x -
              cos_theta * ivEnvironmentParams.foot_origin_shift.y;

  marker->pose.position.x = foot_pose.getX() + x_shift;
  marker->pose.position.y = foot_pose.getY() + y_shift;
  marker->pose.position.z = foot_pose.getZ() + ivEnvironmentParams.foot_size.z / 2.0;
  marker->pose.orientation = tf::createQuaternionMsgFromYaw(foot_pose.getYaw());

  marker->scale.x = ivEnvironmentParams.foot_size.x; // - 0.01;
  marker->scale.y = ivEnvironmentParams.foot_size.y; // - 0.01;
  marker->scale.z = ivEnvironmentParams.foot_size.z;

  // TODO: make color configurable?
  if (foot_pose.getLeg() == RIGHT)
  {
    marker->color.r = 0.0f;
    marker->color.g = 1.0f;
  }
  else // leg == LEFT
  {
    marker->color.r = 1.0f;
    marker->color.g = 0.0f;
  }
  marker->color.b = 0.0;
  marker->color.a = 0.6;

  marker->lifetime = ros::Duration();
}

void FootstepPlanner::publishParams()
{
  flor_footstep_planner_msgs::FootstepPlannerParams params;

  params.change_mask = 0;

  // set step cost type
  params.change_mask |= flor_footstep_planner_msgs::FootstepPlannerParams::STEP_COST_ESTIMATOR;
  if (ivEnvironmentParams.step_cost_estimator_type_name == "EuclideanStepCostEstimator")
    params.step_cost_type = EUCLIDEAN_STEP_COST;
  else if (ivEnvironmentParams.step_cost_estimator_type_name == "GprStepCostEstimator")
    params.step_cost_type = GPR_STEP_COST;
  else if (ivEnvironmentParams.step_cost_estimator_type_name == "MapStepCostEstimator")
    params.step_cost_type = MAP_STEP_COST;
  else if (ivEnvironmentParams.step_cost_estimator_type_name == "BoundaryStepCostEstimator")
    params.step_cost_type = BOUNDARY_STEP;
  else if (ivEnvironmentParams.step_cost_estimator_type_name == "DynamicsStepCostEstimator")
    params.step_cost_type = DYNAMICS_STEP;

  // set collision mode
  params.change_mask |= flor_footstep_planner_msgs::FootstepPlannerParams::COLLISION_CHECK_TYPE;
  params.collision_check_type = ivEnvironmentParams.collision_check_type;

  // set foot size
  params.change_mask |= flor_footstep_planner_msgs::FootstepPlannerParams::FOOT_SIZE;
  params.foot_size = ivEnvironmentParams.foot_size;
  params.foot_origin_shift = ivEnvironmentParams.foot_origin_shift;
  params.foot_seperation = ivEnvironmentParams.foot_seperation;

  // set upper body dimensions
  params.change_mask |= flor_footstep_planner_msgs::FootstepPlannerParams::UPPER_BODY_SIZE;
  params.upper_body_size = ivEnvironmentParams.upper_body_size;
  params.upper_body_origin_shift = ivEnvironmentParams.upper_body_origin_shift;

  // standard step params
  params.change_mask |= flor_footstep_planner_msgs::FootstepPlannerParams::STANDARD_STEP_PARAMS;
  params.step_duration = ivEnvironmentParams.step_duration;
  params.sway_duration = ivEnvironmentParams.sway_duration;
  params.swing_height = ivEnvironmentParams.swing_height;
  params.lift_height = ivEnvironmentParams.lift_height;

  // terrain model
  params.change_mask |= flor_footstep_planner_msgs::FootstepPlannerParams::TERRAIN_MODEL;
  params.use_terrain_model = ivEnvironmentParams.use_terrain_model;
  params.min_sampling_steps_x = ivEnvironmentParams.foot_contact_min_sampling_steps_x;
  params.min_sampling_steps_y = ivEnvironmentParams.foot_contact_min_sampling_steps_y;
  params.max_sampling_steps_x = ivEnvironmentParams.foot_contact_max_sampling_steps_x;
  params.max_sampling_steps_y = ivEnvironmentParams.foot_contact_max_sampling_steps_y;
  params.max_intrusion_z = ivEnvironmentParams.foot_contact_max_intrusion_z;
  params.max_ground_clearance = ivEnvironmentParams.foot_contact_max_ground_clearance;
  params.minimal_support = ivEnvironmentParams.foot_contact_minimal_support;

  ivCurrentParamsPub.publish(params);
}
}
