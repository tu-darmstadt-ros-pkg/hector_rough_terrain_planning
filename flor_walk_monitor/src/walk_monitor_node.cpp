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
  transform_pose_sub_ = nh.subscribe("/flor/state/bdi_to_world_pose", 1, &WalkMonitorLogger::setTransformPoseBdiToWorld, &(*logger));

  // publish topics
  footstep_plan_walk_pub = nh.advertise<flor_footstep_planner_msgs::AtlasWalkFootstepPlan>("/flor/footstep_planner/footstep_plan_walk", 2);
  footstep_plan_step_pub = nh.advertise<flor_footstep_planner_msgs::AtlasStepFootstepPlan>("/flor/footstep_planner/footstep_plan_step", 2);
  footstep_plan_vis_pub = nh_.advertise<visualization_msgs::MarkerArray>("/flor/walk_monitor/footsteps_array", 2);
  feet_poses_start_vis_pub = nh_.advertise<visualization_msgs::MarkerArray>("/flor/walk_monitor/footsteps_start_array", 2);
  footstep_path_vis_pub = nh_.advertise<nav_msgs::Path>("/flor/walk_monitor/path", 1);
  footstep_path_body_vis_pub = nh_.advertise<visualization_msgs::MarkerArray>("/flor/walk_monitor/footsteps_path_body_array", 2);
  walk_performance_pub = nh_.advertise<walk_performance>("/flor/walk_monitor/walk_performance", 1);
}

WalkMonitorNode::~WalkMonitorNode()
{
}

void WalkMonitorNode::reset()
{
  logger->reset();
  walk_started = false;
  walk_finished = false;
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
  //setFootstepPlan(*footstep_plan, flor_footstep_planner_msgs::FootstepPlan::MODE_WALK);
}

void WalkMonitorNode::setStepFootstepPlan(const flor_footstep_planner_msgs::AtlasStepFootstepPlanConstPtr &footstep_plan)
{
  saveLogToDBAndReset();
  //setFootstepPlan(*footstep_plan, flor_footstep_planner_msgs::FootstepPlan::MODE_STEP);
}

void WalkMonitorNode::setFootstepStart(const flor_footstep_planner_msgs::FeetPosesConstPtr &feet_start_poses)
{
  flor_footstep_planner_msgs::FeetPoses feet = *feet_start_poses;
  logger->setFootstepStart(feet);
  footstep_plan_vis->publishFeetStartPosesVis(feet_poses_start_vis_pub, *(logger->getFeetStartPoses()));
}

void WalkMonitorNode::saveLogToDB()
{
  // save current log
  if (mongodb && !logger->empty() && walk_started)
  {
    walk_performance data;
    //logger->getWalkPerformanceData(data);
    if (!mongodb->insert(data))
      ROS_ERROR("Can't insert performance data to database.");
    else
      ROS_INFO("Log inserted sucessfully into database.");

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
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flor_walk_monitor");

  flor_walk_monitor::WalkMonitorNode monitorNode;

  ros::spin();

  return 0;
}
