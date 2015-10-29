#include <flor_walk_monitor/walk_monitor_vis_node.h>

namespace flor_walk_monitor
{
WalkMonitorVisNode::WalkMonitorVisNode()
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");

  // load db settings
  std::string db_name;
  std::string db_host;
  unsigned int db_port;

  nh_.param("mongodb/db_name", db_name, std::string(""));
  nh_.param("mongodb/db_host", db_host, std::string(""));
  nh_.param("mongodb/db_port", (int&)db_port, 0);

  // init db connection
  try
  {
    mongodb.reset(new WalkMonitorMongodb(db_name, db_host, db_port));
  }
  catch (mongo::DBException e)
  {
    ROS_ERROR("Can't connect to mongo db: %s", e.what());
    exit(1);
  }

  footstep_planner_vis.reset(new flor_footstep_plan_vis::FootstepPlannerVis(nh_));

  // publish topics
  footstep_plan_vis_pub = nh_.advertise<visualization_msgs::MarkerArray>("/flor/walk_monitor/vis/footstep_plan", 1);
  feet_poses_start_vis_pub = nh_.advertise<visualization_msgs::MarkerArray>("/flor/walk_monitor/vis/footsteps_start_array", 1);
  footstep_path_vis_pub = nh_.advertise<nav_msgs::Path>("/flor/walk_monitor/vis/footstep_path", 1);
  zmp_vis_pub = nh_.advertise<nav_msgs::Path>("/flor/walk_monitor/vis/zmp", 1);
  cop_vis_pub = nh_.advertise<nav_msgs::Path>("/flor/walk_monitor/vis/cop", 1);
  cop_posn_left_vis_pub = nh_.advertise<nav_msgs::Path>("/flor/walk_monitor/vis/cop/posn/left", 1);
  cop_posn_right_vis_pub = nh_.advertise<nav_msgs::Path>("/flor/walk_monitor/vis/cop/posn/right", 1);
  cop_force_left_vis_pub = nh_.advertise<nav_msgs::Path>("/flor/walk_monitor/vis/cop/force/left", 1);
  cop_force_right_vis_pub = nh_.advertise<nav_msgs::Path>("/flor/walk_monitor/vis/cop/force/right", 1);

  // start oneshot timer
  timer = nh_.createTimer(ros::Duration(0.1), &WalkMonitorVisNode::loadAndVisData, this, true);
}

WalkMonitorVisNode::~WalkMonitorVisNode()
{
}

void WalkMonitorVisNode::loadAndVisData(const ros::TimerEvent &/*timer_event*/)
{
  std::list<walk_performance> data;
  mongodb->getAllData(data);

  if (!data.empty())
    publishVis(data.front());
}

void WalkMonitorVisNode::publishVis(const flor_walk_monitor::walk_performance &data)
{
//  if (data_set.target_points.size() == 0)
//    return;

//  if (data_set.target_points[0].is_fallen_over)
//    ROS_WARN("Robot has fallen in this trial!");

  publishFootstepVis(data);
  publishPathVis(data);

//  publishPointVecVis(zmp_vis_pub, data_set, ZMP);
//  publishPointVecVis(cop_vis_pub, data_set, CoP);
//  publishPointVecVis(cop_posn_left_vis_pub, data_set, CoP_posn_left);
//  publishPointVecVis(cop_posn_right_vis_pub, data_set, CoP_posn_right);
//  publishPointVecVis(cop_force_left_vis_pub, data_set, CoP_force_left);
//  publishPointVecVis(cop_force_right_vis_pub, data_set, CoP_force_right);
}

void WalkMonitorVisNode::publishFootstepVis(const walk_performance &data)
{
  //footstep_planner_vis->publishFootstepPlanVis(footstep_plan_vis_pub, data.planned_footsteps);
  footstep_planner_vis->publishFeetStartPosesVis(feet_poses_start_vis_pub, data.feet_start_poses);
}

void WalkMonitorVisNode::publishPathVis(const walk_performance &data)
{
  //footstep_planner_vis->publishPathVis(footstep_path_vis_pub, data.planned_footsteps);
}

void WalkMonitorVisNode::publishPointVecVis(ros::Publisher &pub, /*const footstep_planner_gpr_data_set &data_set,*/ data_type select)
{
  if (pub.getNumSubscribers() > 0)
  {
    // extract points
    std::vector<geometry_msgs::Point> point_vec;
    //extractPointVec(data_set, select, point_vec);

    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped pose;

    // convert to msg
    path_msg.header.stamp = ros::Time::now();
    //path_msg.header.frame_id = data_set.header.frame_id;
    pose.header = path_msg.header;

    for (std::vector<geometry_msgs::Point>::const_iterator itr = point_vec.begin(); itr != point_vec.end(); itr++)
    {
      pose.pose.position = *itr;
      path_msg.poses.push_back(pose);
    }

    pub.publish(path_msg);
  }
}

void WalkMonitorVisNode::point32ToPoint(const geometry_msgs::Point32 &point32, geometry_msgs::Point &point) const
{
  point.x = point32.x;
  point.y = point32.y;
  point.z = point32.z;
}

void WalkMonitorVisNode::extractPointVec() const
{
  //point_vec.clear();

//  if (data_set.data_points.size() == 0)
//    return;

//  for (unsigned int i = 1; i < data_set.data_points.size(); i++)
//  {
//    geometry_msgs::Point p;

    // get point
//    switch (select)
//    {
//      case ZMP: point32ToPoint(data_set.target_points[i].ZMP, p); break;
//      case CoP: point32ToPoint(data_set.target_points[i].CoP, p); break;
//      case CoP_posn_left: point32ToPoint(data_set.target_points[i].CoP_posn_left, p); break;
//      case CoP_posn_right: point32ToPoint(data_set.target_points[i].CoP_posn_right, p); break;
//      case CoP_force_left: point32ToPoint(data_set.target_points[i].CoP_force_left, p); break;
//      case CoP_force_right: point32ToPoint(data_set.target_points[i].CoP_force_right, p); break;
//    }

    // transform points
//    switch (select)
//    {
//      case ZMP:
//      case CoP:
//      {
//        p.x += data_set.target_points[i].pos_est.position.x;
//        p.y += data_set.target_points[i].pos_est.position.y;
//        p.z += data_set.target_points[i].pos_est.position.z;
//        break;
//      }
//      case CoP_posn_left:
//      {
//        p.x += data_set.data_points[i-1].left_foot.pose.position.x;
//        p.y += data_set.data_points[i-1].left_foot.pose.position.y;
//        p.z += data_set.data_points[i-1].left_foot.pose.position.z + 0.080864; // TODO: data seems to have an offset
//        break;
//      }
//      case CoP_posn_right:
//      {
//        p.x += data_set.data_points[i-1].right_foot.pose.position.x;
//        p.y += data_set.data_points[i-1].right_foot.pose.position.y;
//        p.z += data_set.data_points[i-1].right_foot.pose.position.z + 0.080864; // TODO: data seems to have an offset
//        break;
//      }
//      case CoP_force_left:
//      case CoP_force_right:
//      {
//        /// @TODO
//        break;
//      }
//    }

//    point_vec.push_back(p);
//  }
}

//void WalkMonitorVisNode::extractFootstepPlan(std::vector<atlas_msgs::AtlasBehaviorStepData> &footstep_plan) const
//{
//  footstep_plan.clear();

//  if (data_set.data_points.size() == 0)
//    return;

//  if (data_set.data_points.begin()->swing_foot.foot_index == LEFT)
//  {
//    footstep_plan.push_back(data_set.data_points[0].left_foot);
//    footstep_plan.push_back(data_set.data_points[0].right_foot);
//  }
//  else
//  {
//    footstep_plan.push_back(data_set.data_points[0].right_foot);
//    footstep_plan.push_back(data_set.data_points[0].left_foot);
//  }
//  for (unsigned int i = 0; i < data_set.data_points.size(); i++)
//    footstep_plan.push_back(data_set.data_points[i].swing_foot);
//}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flor_walk_monitor_vis");

  flor_walk_monitor::WalkMonitorVisNode monitorVisNode;

  ros::spin();

  return 0;
}
