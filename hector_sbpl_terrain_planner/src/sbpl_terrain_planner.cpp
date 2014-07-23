/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Mike Phillips
*********************************************************************/

#include <hector_sbpl_terrain_planner/sbpl_terrain_planner.h>
//#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
//#include <sbpl_lattice_planner/SBPLTerrainPlannerStats.h>
#include <hector_sbpl_terrain_planner/discrete_space_information/environment_navxytheta_stability_lat.h>

using namespace std;
using namespace ros;


//PLUGINLIB_DECLARE_CLASS(sbpl_latice_planner, SBPLTerrainPlanner, sbpl_lattice_planner::SBPLTerrainPlanner, nav_core::BaseGlobalPlanner);

namespace sbpl_terrain_planner{

class LatticeSCQ : public StateChangeQuery{
  public:
    LatticeSCQ(EnvironmentNAVXYTHETALAT* env, std::vector<nav2dcell_t> const & changedcellsV)
      : env_(env), changedcellsV_(changedcellsV) {
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getPredecessors() const{
      if(predsOfChangedCells_.empty() && !changedcellsV_.empty())
        env_->GetPredsofChangedEdges(&changedcellsV_, &predsOfChangedCells_);
      return &predsOfChangedCells_;
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getSuccessors() const{
      if(succsOfChangedCells_.empty() && !changedcellsV_.empty())
        env_->GetSuccsofChangedEdges(&changedcellsV_, &succsOfChangedCells_);
      return &succsOfChangedCells_;
    }

    EnvironmentNAVXYTHETALAT * env_;
    std::vector<nav2dcell_t> const & changedcellsV_;
    mutable std::vector<int> predsOfChangedCells_;
    mutable std::vector<int> succsOfChangedCells_;
};

SBPLTerrainPlanner::SBPLTerrainPlanner()
  : initialized_(false){//, costmap_ros_(NULL){
}

SBPLTerrainPlanner::SBPLTerrainPlanner(std::string name)
  : initialized_(false){//, costmap_ros_(NULL){
  initialize(name);
}

void SBPLTerrainPlanner::mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
{
    t_lastMapPos_ = map->header.stamp;
    map_center_map=map->info.origin.position;
}

void SBPLTerrainPlanner::initialize(std::string name){//, costmap_2d::Costmap2DROS* costmap_ros){
  if(!initialized_){
    ros::NodeHandle private_nh("~/"+name);
    ros::NodeHandle nh("~/");
    ros::NodeHandle n("");

    ROS_INFO("Name is %s", name.c_str());
    tf_listener_.reset(new tf::TransformListener());

    private_nh.param("planner_type", planner_type_, string("ARAPlanner"));
    private_nh.param("allocated_time", allocated_time_, 15.0);
    private_nh.param("initial_epsilon",initial_epsilon_,3.0);
    nh.param("environment_type", environment_type_, string("testXYThetaLattice"));
    private_nh.param("forward_search", forward_search_, bool(false));
    private_nh.param("primitive_filename",primitive_filename_,string(""));
    private_nh.param("force_scratch_limit",force_scratch_limit_,500);

    double nominalvel_mpersecs, timetoturn45degsinplace_secs;
    private_nh.param("nominalvel_mpersecs", nominalvel_mpersecs, 0.4);
    private_nh.param("timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);

    int lethal_obstacle;
    private_nh.param("lethal_obstacle",lethal_obstacle,20);
    lethal_obstacle_ = (unsigned char) lethal_obstacle;
    inscribed_inflated_obstacle_ = lethal_obstacle_-1;
    sbpl_cost_multiplier_ = (unsigned char) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE/inscribed_inflated_obstacle_ + 1);
    ROS_DEBUG("SBPL: lethal: %uz, inscribed inflated: %uz, multiplier: %uz",lethal_obstacle,inscribed_inflated_obstacle_,sbpl_cost_multiplier_);
    map_subscriber_=n.subscribe("map", 1000,  &SBPLTerrainPlanner::mapCallback, this);

//    if ("XYThetaLattice" == environment_type_ || "testXYThetaLattice" == environment_type_){
//      ROS_DEBUG("Using a 3D costmap for theta lattice\n");
//      env_ = new EnvironmentNAVXYTHETALAT();
//    }
//    else if ("XYThetaStability" == environment_type_){
        ROS_DEBUG("Using a 3D costmap for theta stab\n");
        env_ = new EnvironmentNAVXYTHETASTAB();
//      }
//    else{
//        ROS_ERROR("XYThetaLattice is currently the only supported environment! Type is:\n");
//        ROS_ERROR("%s",environment_type_.c_str());
//      exit(1);
//
    int obst_cost_thresh = 1;//costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE);

    vector<sbpl_2Dpt_t> perimeterptsV;
    perimeterptsV.reserve(4);

    sbpl_2Dpt_t pt;
    pt.x = 0.1;
    pt.y = 0.1;
    perimeterptsV.push_back(pt);
    pt.x = -0.1;
    perimeterptsV.push_back(pt);
    pt.y = -0.1;
    perimeterptsV.push_back(pt);
    pt.x = 0.1;
    perimeterptsV.push_back(pt);


    bool ret;
    try{
      ret = env_->InitializeEnv(1024, // width
                                1024, // height
                                0, // mapdata
                                0, 0, 0, // start (x, y, theta, t)
                                0, 0, 0, // goal (x, y, theta)
                                0, 0, 0, //goal tolerance
                                perimeterptsV, 0.05, nominalvel_mpersecs,
                                timetoturn45degsinplace_secs, obst_cost_thresh,
                                primitive_filename_.c_str());
    }
    catch(SBPL_Exception e){
      ROS_ERROR("SBPL encountered a fatal exception!");
      ret = false;
    }
    if(!ret){
      ROS_ERROR("SBPL initialization failed!");
      exit(1);
    }

    for (ssize_t ix(0); ix < 1024; ++ix){
      for (ssize_t iy(0); iy < 1024; ++iy){
        //env_->UpdateCost(ix, iy, costMapCostToSBPLCost(cost_map_.getCost(ix,iy)));
        env_->UpdateCost(ix, iy, 0);
      }
    }


    if ("ARAPlanner" == planner_type_){
      ROS_INFO("Planning with ARA*");
      planner_ = new ARAPlanner(env_, forward_search_);
    }
    else if ("ADPlanner" == planner_type_){
      ROS_INFO("Planning with AD*");
      planner_ = new ADPlanner(env_, forward_search_);
    }
    else{
      ROS_ERROR("ARAPlanner and ADPlanner are currently the only supported planners!\n");
      exit(1);
    }


    ROS_INFO("[sbpl_lattice_planner] Initialized successfully");
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    //stats_publisher_ = private_nh.advertise<sbpl_lattice_planner::SBPLTerrainPlannerStats>("sbpl_lattice_planner_stats", 1);

    initialized_ = true;
  }
}

//Taken from Sachin's sbpl_cart_planner
//This rescales the costmap according to a rosparam which sets the obstacle cost
unsigned char SBPLTerrainPlanner::costMapCostToSBPLCost(unsigned char newcost){
  if(newcost == costmap_2d::LETHAL_OBSTACLE)
    return lethal_obstacle_;
  else if(newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    return inscribed_inflated_obstacle_;
  else if(newcost == 0 || newcost == costmap_2d::NO_INFORMATION)
    return 0;
  else
    return (unsigned char) (newcost/sbpl_cost_multiplier_ + 0.5);
}

void SBPLTerrainPlanner::publishStats(int solution_cost, int solution_size,
                                      const geometry_msgs::PoseStamped& start,
                                      const geometry_msgs::PoseStamped& goal){

}

bool SBPLTerrainPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan){
  if(!initialized_){
    ROS_ERROR("Global planner is not initialized");
    return false;
  }



  plan.clear();

  ROS_DEBUG("[sbpl_lattice_planner] getting fresh copy of costmap");
  //costmap_ros_->clearRobotFootprint();
  ROS_DEBUG("[sbpl_lattice_planner] robot footprint cleared");

  //costmap_ros_->getCostmapCopy(cost_map_);

/*     tf::StampedTransform worldTosensorTf;

  try{
     tf_listener_->waitForTransform("/map", "/base_link",  t_lastMapPos_, ros::Duration(0.6));
     tf_listener_->lookupTransform("/map", "/base_link", t_lastMapPos_, worldTosensorTf);
     tf::Vector3 org=worldTosensorTf.getOrigin();
     ROS_INFO("[hector_sbpl_terrain_planner] getting start point (%f,%f)",org.x(),org.y());
   }catch(tf::TransformException& ex){
     ROS_ERROR_STREAM( "[hector_sbpl_terrain_planner] Transform error for map-base_link TF: " << ex.what() << "\n");
  }*/

  ROS_INFO("[hector_sbpl_terrain_planner] getting start point (%g,%g) goal point (%g,%g)",
           start.pose.position.x, start.pose.position.y,goal.pose.position.x, goal.pose.position.y);
  double theta_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
  double theta_goal = 2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);

  try{
 /*     unsigned int startx;
      unsigned int starty;
     env_->mapToGridMap(start.pose.position.x, start.pose.position.y, startx, starty);
     int ret = env_->SetStart(startx, starty, theta_start);*/
    int ret = env_->SetStart(start.pose.position.x  , start.pose.position.y , theta_start);
    if(ret < 0 || planner_->set_start(ret) == 0){
      ROS_ERROR("ERROR: failed to set start state\n");
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
    return false;
  }

  try{
 /*     unsigned int goalx;
      unsigned int goaly;
      env_->mapToGridMap(goal.pose.position.x, goal.pose.position.y, goalx, goaly);
      int ret = env_->SetGoal(goalx, goaly, theta_goal);*/
    int ret = env_->SetGoal(goal.pose.position.x , goal.pose.position.y , theta_goal);
    if(ret < 0 || planner_->set_goal(ret) == 0){
      ROS_ERROR("ERROR: failed to set goal state\n");
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
    return false;
  }

  int offOnCount = 0;
  int onOffCount = 0;
  int allCount = 0;
  vector<nav2dcell_t> changedcellsV;


  try{
    if(!changedcellsV.empty()){
      StateChangeQuery* scq = new LatticeSCQ(env_, changedcellsV);
      planner_->costs_changed(*scq);
      delete scq;
    }

    if(allCount > force_scratch_limit_)
      planner_->force_planning_from_scratch();
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL failed to update the costmap");
    return false;
  }

  //setting planner parameters
  ROS_DEBUG("allocated:%f, init eps:%f\n",allocated_time_,initial_epsilon_);
  planner_->set_initialsolution_eps(initial_epsilon_);
  planner_->set_search_mode(false);


  ROS_DEBUG("[sbpl_lattice_planner] run planner");
  vector<int> solution_stateIDs;
  int solution_cost;
  try{
      ROS_INFO("replan");
    int ret = planner_->replan(allocated_time_, &solution_stateIDs, &solution_cost);
    if(ret){
        ROS_INFO("Solution is found");
      ROS_DEBUG("Solution is found\n");}
    else{
      ROS_INFO("Solution not found\n");
      publishStats(solution_cost, 0, start, goal);
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while planning");
    return false;
  }

  ROS_INFO("size of solution=%d", (int)solution_stateIDs.size());

  vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_path;
  try{
    env_->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path);
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while reconstructing the path");
    return false;
  }
  ros::Time plan_time = ros::Time::now();

  ROS_INFO("Plan has %d points.\n", (int)sbpl_path.size());

  //create a message for the plan
  nav_msgs::Path gui_path;
  gui_path.poses.resize(sbpl_path.size());
  //gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  gui_path.header.frame_id = "map";
  gui_path.header.stamp = plan_time;
  for(unsigned int i=0; i<sbpl_path.size(); i++){
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = gui_path.header.frame_id = "map";


    pose.pose.position.x = sbpl_path[i].x;
    pose.pose.position.y = sbpl_path[i].y;

    pose.pose.position.z = start.pose.position.z;

    tf::Quaternion temp;
    temp.setEulerZYX(sbpl_path[i].theta,0,0);
    pose.pose.orientation.x = temp.getX();
    pose.pose.orientation.y = temp.getY();
    pose.pose.orientation.z = temp.getZ();
    pose.pose.orientation.w = temp.getW();

    plan.push_back(pose);

    gui_path.poses[i].pose.position.x = plan[i].pose.position.x;
    gui_path.poses[i].pose.position.y = plan[i].pose.position.y;
    gui_path.poses[i].pose.position.z = plan[i].pose.position.z;
  }
  plan_pub_.publish(gui_path);
  publishStats(solution_cost, sbpl_path.size(), start, goal);


  return true;
}


};
