//=================================================================================================
// Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt
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


#include "ros/ros.h"
#include <hector_sbpl_terrain_planner/sbpl_terrain_planner.h>
#include <nav_msgs/Path.h>

//#include <costmap_2d/costmap_2d_ros.h>
//#include <hector_nav_msgs/GetRobotTrajectory.h>

class PlannerTest
{
public:
  PlannerTest()
  {
    path_.header.frame_id = "map";

    ros::NodeHandle nh;

    //costmap_2d_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tfl_);

    planner_ = new sbpl_terrain_planner::SBPLTerrainPlanner();
    planner_->initialize("hector_sbpl_terrain_planner");


    exploration_plan_pub_ = nh.advertise<nav_msgs::Path>("exploration_path",2);
    search_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/gen_search_pose",2);

    //plan_cmd_sub_ = nh.subscribe("/replan", 2, &PlannerTest::replanCallback, this);
    goal_pose_sub_ = nh.subscribe("/goal", 2, &PlannerTest::goalPoseCallback, this);
    //search_pose_sub_ = nh.subscribe("/search_pose", 2, &PlannerTest::searchPoseCallback, this);
  }

  void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    ROS_INFO("Goal pose callback");

    tf::Stamped<tf::Pose> robot_pose_tf;
    //costmap_2d_ros_->getRobotPose(robot_pose_tf);

    //Start always at 0 for now;
    tf::Pose start_pose;
    start_pose.setIdentity();
    robot_pose_tf.setData(start_pose);

    geometry_msgs::PoseStamped pose;
    tf::poseStampedTFToMsg(robot_pose_tf, pose);
    planner_->makePlan(pose, *msg, path_.poses);

    path_.header.stamp = ros::Time::now();

    if (exploration_plan_pub_.getNumSubscribers() > 0)
    {
      exploration_plan_pub_.publish(path_);
    }
  }

  /*
  void searchPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    ROS_INFO("Search pose callback");

    tf::Stamped<tf::Pose> robot_pose_tf;
    costmap_2d_ros_->getRobotPose(robot_pose_tf);

    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped pose_new;
    tf::poseStampedTFToMsg(robot_pose_tf, pose);
    //planner_->getObservationPose(*msg,0.5, pose_new);

    if (search_pose_pub_.getNumSubscribers() > 0)
    {
      search_pose_pub_.publish(pose_new);
    }
  }
  */

  /*
  void replanCallback(const std_msgs::Empty& msg)
  {
    ROS_INFO("Explore callback");

    if (exploration_plan_pub_.getNumSubscribers() > 0)
    {
      tf::Stamped<tf::Pose> robot_pose_tf;
      costmap_2d_ros_->getRobotPose(robot_pose_tf);

      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(robot_pose_tf, pose);

      planner_->doExploration(pose, path_.poses);
      path_.header.stamp = ros::Time::now();

      exploration_plan_pub_.publish(path_);
    }
  }
  */

protected:
  //hector_exploration_planner::HectorExplorationPlanner* planner_;

  sbpl_terrain_planner::SBPLTerrainPlanner* planner_;

  ros::ServiceServer exploration_plan_service_server_;

  costmap_2d::Costmap2DROS* costmap_2d_ros_;
  //tf::TransformListener tfl_;

  nav_msgs::Path path_;

  ros::Publisher exploration_plan_pub_;
  ros::Publisher search_pose_pub_;

  ros::Subscriber plan_cmd_sub_;
  ros::Subscriber goal_pose_sub_;
  ros::Subscriber search_pose_sub_;

};

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  PlannerTest pt;

  ros::spin();

  return 0;
}
