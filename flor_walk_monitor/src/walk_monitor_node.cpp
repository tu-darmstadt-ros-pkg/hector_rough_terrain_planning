#include <flor_walk_monitor/walk_monitor_node.h>

namespace flor_walk_monitor
{
WalkMonitorNode::WalkMonitorNode()
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");

  footstep_plan_vis.reset(new flor_footstep_plan_vis::FootstepPlannerVis(nh_));

  // load logger parameters
  logger.reset(new WalkMonitorLogger());

  double contact_force_z_thresh;
  nh_.param("logger/contact_force_z_thresh", contact_force_z_thresh, 0.0);
  logger->setContactForceZThresh(contact_force_z_thresh);

  // load monitoring parameters
  nh_.param("monitor/max_step_difference", max_step_diff, 0.1);
  nh_.param("monitor/max_yaw_difference", max_yaw_diff, 0.1);

  // load db settings
  bool use_mongo_db = false;
  std::string db_name;
  std::string db_host;
  unsigned int db_port;
  bool drop_old;

  nh_.param("use_mongo_db", use_mongo_db, false);
  nh_.param("mongodb/db_name", db_name, std::string(""));
  nh_.param("mongodb/db_host", db_host, std::string(""));
  nh_.param("mongodb/db_port", (int&)db_port, 0);
  nh_.param("mongodb/drop_old", drop_old, false);

  if (use_mongo_db)
  {
    // init db connection
    try
    {
      if (drop_old)
      {
        mongo_ros::dropDatabase(db_name, db_host, db_port, 60.0);
        ROS_WARN("Old 'database %s@%s:%u'' dropped!", db_name.c_str(), db_host.c_str(), db_port);
      }
      mongodb.reset(new WalkMonitorMongodb(db_name, db_host, db_port));
    }
    catch (mongo::DBException e)
    {
      ROS_ERROR("Can't connect to mongo db, no logging will be available: %s", e.what());
      mongodb.reset();
    }
  }

  // subscribe topics, ensure that all data is given in world frame
  footstep_planner_params_sub = nh.subscribe("/flor/footstep_planner/set_params", 1, &WalkMonitorNode::setFootstepPlannerParams, this);
  footstep_plan_walk_sub = nh.subscribe("/flor/footstep_planner/footstep_plan_walk", 1, &WalkMonitorNode::setWalkFootstepPlan, this);
  footstep_plan_step_sub = nh.subscribe("/flor/footstep_planner/footstep_plan_step", 1, &WalkMonitorNode::setStepFootstepPlan, this);
  footstep_start_sub = nh.subscribe("/flor/footstep_planner/footstep_start", 1, &WalkMonitorNode::setFootstepStart, this);
  atlas_sim_interface_state_sub = nh.subscribe("/atlas/atlas_sim_interface_state", 1, &WalkMonitorNode::setAtlasSimInterfaceState, this);
  atlas_walk_feedback_sub = nh.subscribe("/flor/controller/atlas_walk_feedback", 1, &WalkMonitorNode::setAtlasBehaviorWalkFeedback, this);
  atlas_step_feedback_sub = nh.subscribe("/flor/controller/atlas_step_feedback", 1, &WalkMonitorNode::setAtlasBehaviorStepFeedback, this);
  controller_mode_sub = nh.subscribe("/flor/controller/mode", 1, &WalkMonitorLogger::setControlMode, &(*logger));
  controller_stability_sub = nh.subscribe("/flor/controller/stability", 1, &WalkMonitorLogger::setRobotStability, &(*logger));
  transform_pose_sub_ = nh.subscribe("/flor/state/bdi_to_world_pose", 1, &WalkMonitorLogger::setTransformPoseBdiToWorld, &(*logger));

  // publish topics
  footstep_plan_walk_pub = nh.advertise<flor_footstep_planner_msgs::AtlasWalkFootstepPlan>("/flor/footstep_planner/footstep_plan_walk", 2);
  footstep_plan_step_pub = nh.advertise<flor_footstep_planner_msgs::AtlasStepFootstepPlan>("/flor/footstep_planner/footstep_plan_step", 2);
  footstep_plan_vis_pub = nh_.advertise<visualization_msgs::MarkerArray>("/flor/walk_monitor/footsteps_array", 2);
  feet_poses_start_vis_pub = nh_.advertise<visualization_msgs::MarkerArray>("/flor/walk_monitor/footsteps_start_array", 2);
  footstep_path_vis_pub = nh_.advertise<nav_msgs::Path>("/flor/walk_monitor/path", 1);
  footstep_path_body_vis_pub = nh_.advertise<visualization_msgs::MarkerArray>("/flor/walk_monitor/footsteps_path_body_array", 2);
  bdi_footsteps_vis_pub = nh_.advertise<visualization_msgs::MarkerArray>("/flor/walk_monitor/bdi_footsteps", 5);
  real_footsteps_vis_pub = nh_.advertise<visualization_msgs::MarkerArray>("/flor/walk_monitor/real_footsteps", 5);
  walk_performance_pub = nh_.advertise<walk_performance>("/flor/walk_monitor/walk_performance", 1);
  walk_monitor_status_pub = nh_.advertise<flor_ocs_msgs::OCSRobotStatus>("/flor/walk_monitor/status", 10);
}

WalkMonitorNode::~WalkMonitorNode()
{
}

void WalkMonitorNode::reset()
{
  logger->reset();
  walk_started = false;
  walk_finished = false;

  publishOCSRobotStatus(RobotStatusCodes::WALK_MONITOR_STATUS, RobotStatusCodes::OK);
  publishOCSRobotStatus(RobotStatusCodes::WALK_MONITOR_WALK_HEALTH, RobotStatusCodes::OK);
}

void WalkMonitorNode::setFootstepPlannerParams(const flor_footstep_planner_msgs::FootstepPlannerParams::ConstPtr &params)
{
  // set foot size
  if (params->change_mask & flor_footstep_planner_msgs::FootstepPlannerParams::FOOT_SIZE)
  {
    footstep_plan_vis->setFootSize(params->foot_size);
    footstep_plan_vis->setFootOriginShift(params->foot_origin_shift);
    footstep_plan_vis->setFootSeperation(params->foot_seperation);
  }

  // set upper body dimensions
  if (params->change_mask & flor_footstep_planner_msgs::FootstepPlannerParams::UPPER_BODY_SIZE)
  {
    footstep_plan_vis->setUpperBodySize(params->upper_body_size);
    footstep_plan_vis->setUpperBodyOriginShift(params->upper_body_origin_shift);
  }
}

void WalkMonitorNode::setWalkFootstepPlan(const flor_footstep_planner_msgs::AtlasWalkFootstepPlanConstPtr &footstep_plan)
{
  saveLogToDBAndReset();
  setFootstepPlan(*footstep_plan, flor_footstep_planner_msgs::FootstepPlan::MODE_WALK);
}

void WalkMonitorNode::setStepFootstepPlan(const flor_footstep_planner_msgs::AtlasStepFootstepPlanConstPtr &footstep_plan)
{
  saveLogToDBAndReset();
  setFootstepPlan(*footstep_plan, flor_footstep_planner_msgs::FootstepPlan::MODE_STEP);
}

void WalkMonitorNode::setFootstepStart(const flor_footstep_planner_msgs::FeetPosesConstPtr &feet_start_poses)
{
  flor_footstep_planner_msgs::FeetPoses feet = *feet_start_poses;
  logger->setFootstepStart(feet);
  footstep_plan_vis->publishFeetStartPosesVis(feet_poses_start_vis_pub, *(logger->getFeetStartPoses()));
}

void WalkMonitorNode::setAtlasBehaviorWalkFeedback(const flor_atlas_msgs::AtlasBehaviorWalkFeedbackConstPtr &behavior_walk_feedback)
{
  logger->setAtlasBehaviorWalkFeedback(behavior_walk_feedback);
  checkWalkStatus();
}

void WalkMonitorNode::setAtlasBehaviorStepFeedback(const flor_atlas_msgs::AtlasBehaviorStepFeedbackConstPtr &behavior_step_feedback)
{
  logger->setAtlasBehaviorStepFeedback(behavior_step_feedback);
  checkWalkStatus();
}

void WalkMonitorNode::setAtlasSimInterfaceState(const atlas_msgs::AtlasSimInterfaceStateConstPtr &atlas_sim_interface_state)
{
  logger->setAtlasSimInterfaceState(atlas_sim_interface_state);
  checkWalkStatus();
}

void WalkMonitorNode::saveLogToDB()
{
  // save current log
  if (mongodb && !logger->empty() && walk_started)
  {
    walk_performance data;
    logger->getWalkPerformanceData(data);
    if (!mongodb->insert(data))
    {
      ROS_ERROR("Can't insert performance data to database.");
      publishOCSRobotStatus(RobotStatusCodes::WALK_MONITOR_LOGGER, RobotStatusCodes::ERROR);
    }
    else
    {
      ROS_INFO("Log inserted sucessfully into database.");
      publishOCSRobotStatus(RobotStatusCodes::WALK_MONITOR_LOGGER, RobotStatusCodes::OK);
    }

    walk_performance_pub.publish(data);
  }
}

void WalkMonitorNode::saveLogToDBAndReset(const ros::TimerEvent &/*timer_event*/)
{
  saveLogToDB();
  reset();
}

void WalkMonitorNode::checkWalkStatus()
{
  // (re)publish current feedback
  footstep_plan_vis->publishFootstepLogVis(bdi_footsteps_vis_pub, logger->getBdiPlannedFootsteps());
  footstep_plan_vis->publishFootstepLogVis(real_footsteps_vis_pub, logger->getRealFootsteps());
  return;

  if (logger->empty())
  {
    publishOCSRobotStatus(RobotStatusCodes::WALK_MONITOR_STATUS, RobotStatusCodes::OK);
    return;
  }

  if (!logger->getControllerMode() || !logger->getAtlasSimInterfaceState())
  {
    ROS_ERROR("No response from controller yet.");
    publishOCSRobotStatus(RobotStatusCodes::WALK_MONITOR_STATUS, RobotStatusCodes::ERROR);
    return;
  }

  // check if new plan was started to run
  if (logger->getAtlasSimInterfaceState()->current_behavior != flor_control_msgs::FlorControlModeCommand::STAND)
    walk_started = true;

  if (!walk_started || walk_finished)
  {
    publishOCSRobotStatus(RobotStatusCodes::WALK_MONITOR_STATUS, RobotStatusCodes::OK);
    return;
  }

  // check for fall or if robot has finished the planned path
  if (logger->hasFallen() || logger->getAtlasSimInterfaceState()->current_behavior == flor_control_msgs::FlorControlModeCommand::STAND)
  {
    if (logger->hasFallen())
    {
      ROS_WARN("Robot has fallen!");
      publishOCSRobotStatus(RobotStatusCodes::WALK_MONITOR_WALK_HEALTH, RobotStatusCodes::ERROR);
    }
    else
    {
      ROS_INFO("Robot has finished path -> write log data to db.");
      publishOCSRobotStatus(RobotStatusCodes::WALK_MONITOR_WALK_HEALTH, RobotStatusCodes::OK);
    }

    // job is done -> save to db and reset logger; wait to get real pose of last footstep
    ros::NodeHandle nh_("~");
    timer = nh_.createTimer(ros::Duration(1.0), &WalkMonitorNode::saveLogToDBAndReset, this, true);
    walk_finished = true;

    publishOCSRobotStatus(RobotStatusCodes::WALK_MONITOR_STATUS, RobotStatusCodes::OK);
    return;
  }

  // check current saturation
  RobotStatusCodes::StatusLevel walk_monitor_status = RobotStatusCodes::OK;
  RobotStatusCodes::StatusLevel walk_monitor_walk_health = RobotStatusCodes::OK;

  std::map<unsigned int, flor_footstep_planner_msgs::StepTarget> diff;
  diffFootsteps(diff, logger->getPlannedFootsteps(), logger->getBdiPlannedFootsteps());

  double max_step_diff_sqr =  max_step_diff*max_step_diff;
  double max_yaw_diff_sqr =  max_yaw_diff*max_yaw_diff;
  for (std::map<unsigned int, flor_footstep_planner_msgs::StepTarget>::const_iterator itr = diff.begin(); itr != diff.end(); itr++)
  {
    geometry_msgs::Vector3 p = itr->second.foot.position;
    double step_diff_sqr = p.x*p.x + p.y*p.y/* + p.z*p.z*/;
    double yaw_diff_sqr = itr->second.foot.yaw*itr->second.foot.yaw;

    if (step_diff_sqr > max_step_diff_sqr || yaw_diff_sqr > max_yaw_diff_sqr)
    {
      ROS_WARN("Emergency stop: Sending empty footstep plan (%f > %f).", sqrt(step_diff_sqr), sqrt(max_step_diff_sqr));
      /// TODO: check if plan is returned in topic /flor/controller/footstep_plan and triggers log write back
      if (logger->getFootstepPlan()->planning_mode == flor_footstep_planner_msgs::FootstepPlan::MODE_WALK)
        footstep_plan_walk_pub.publish(flor_footstep_planner_msgs::AtlasWalkFootstepPlan());
      else if (logger->getFootstepPlan()->planning_mode == flor_footstep_planner_msgs::FootstepPlan::MODE_STEP)
        footstep_plan_step_pub.publish(flor_footstep_planner_msgs::AtlasStepFootstepPlan());
      else
      {
        ROS_ERROR("checkWalkStatus: Unknown planning mode %u. Fix it immediatly!", logger->getFootstepPlan()->planning_mode);
        walk_monitor_status = RobotStatusCodes::ERROR;
      }

      walk_monitor_walk_health = RobotStatusCodes::ERROR;
      break;
    }
    else if (step_diff_sqr > 0.001 || yaw_diff_sqr > 0.001)
      walk_monitor_walk_health = RobotStatusCodes::WARNING;
  }

  publishOCSRobotStatus(RobotStatusCodes::WALK_MONITOR_STATUS, walk_monitor_status);
  publishOCSRobotStatus(RobotStatusCodes::WALK_MONITOR_WALK_HEALTH, walk_monitor_walk_health);
}

void WalkMonitorNode::publishOCSRobotStatus(const RobotStatusCodes::StatusCode& code, const RobotStatusCodes::StatusLevel& level) const
{
  flor_ocs_msgs::OCSRobotStatus status;
  status.stamp = ros::Time::now();
  status.status = RobotStatusCodes::status(code, level);
  walk_monitor_status_pub.publish(status);
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flor_walk_monitor");

  flor_walk_monitor::WalkMonitorNode monitorNode;

  ros::spin();

  return 0;
}
