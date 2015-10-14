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


#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/intensity_gradient.h>

#include <flor_terrain_classifier/terrain_classifier.h>
#include <flor_terrain_classifier/terrain_model.h>



using namespace flor_terrain_classifier;
using namespace hector_terrain_model;
class TerrainClassifierTest
{
public:
    TerrainClassifierTest()
    {}
    void initialPoseCb(geometry_msgs::PoseWithCovarianceStamped msg);
    void test_terrain_classifier();
};

void TerrainClassifierTest::initialPoseCb(geometry_msgs::PoseWithCovarianceStamped msg)
{
ROS_INFO("reveiced pose");
}

void TerrainClassifierTest::test_terrain_classifier()
{
  ros::NodeHandle nh;
  flor_terrain_classifier::TerrainClassifierParams params(nh);
  ros::Publisher cloud_input_pub = nh.advertise<sensor_msgs::PointCloud2>("/flor/terrain_classifier/cloud_input", 3);
  ros::Subscriber initialpose_sub = nh.subscribe("/initialpose", 100, &TerrainClassifierTest::initialPoseCb, this);

  params.filter_mask = FILTER_PASS_THROUGH | FILTER_VOXEL_GRID | FILTER_MLS_SMOOTH;
  flor_terrain_classifier::TerrainClassifier::Ptr terrain_classifier(new flor_terrain_classifier::TerrainClassifier(params));


  ROS_INFO("Load point cloud");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::io::loadPCDFile("../pointclouds/left_obstacle_start_zero.pcd", *cloud_original);
  //pcl::io::loadPCDFile("../pointclouds/konststeigend_x.pcd", *cloud_original);
  //pcl::io::loadPCDFile("../pointclouds/zwei_ebenen_steigend.pcd", *cloud_original);
  //pcl::io::loadPCDFile("../pointclouds/dach.pcd", *cloud_original);
  //pcl::io::loadPCDFile("../pointclouds/ramp2_filtered.pcd", *cloud_original);
  pcl::io::loadPCDFile("../pointclouds/pc_small_step.pcd", *cloud_original);
  //pcl::io::loadPCDFile("../pointclouds/big_sim.pcd", *cloud_original);



  hector_terrain_model::TerrainModel terrain_model(*cloud_original);


  // add filtered point cloud to classifier
  terrain_classifier->addCloud(cloud_original);


  // detect edges
  ROS_INFO("Compute HeightRating...");
  terrain_classifier->computeHeight();

  // visualization
  pcl::visualization::PCLVisualizer viewer("Terrain classifier");
  viewer.initCameraParameters();

  int view_port_1(0);
  viewer.createViewPort(0.0, 0.5, 0.5, 1.0, view_port_1);
  viewer.addCoordinateSystem(0.5, view_port_1);
  viewer.addPointCloud<pcl::PointXYZ>(terrain_classifier->getCloudInput(), "input cloud", view_port_1);

  int view_port_2(0);
  viewer.createViewPort(0.5, 0.5, 1.0, 1.0, view_port_2);
  viewer.addCoordinateSystem(0.5, view_port_2);

  int view_port_3(0);
  viewer.createViewPort(0.0, 0.0, 0.5, 0.5, view_port_3);
  viewer.addCoordinateSystem(0.5, view_port_3);
  terrain_classifier->showHeight(viewer, "heightDiff", view_port_3);


  int view_port_4(0);
  viewer.createViewPort(0.5, 0.0, 1.0, 0.5, view_port_4);
  viewer.addCoordinateSystem(0.5, view_port_4);

  ROS_INFO("Compute Position rating...");


  // Position, Orientation (in radiants)
  float x, y, orientation;
  //DEBUG for Simulation (file: big_sim.pcd)
  //x  = 1.511102; y = 1.011102; orientation = 0.7854; // bpr is 2.80 this is a normal position
  //x = 1.400002; y = 0.900002; orientation = 0.7854; // convex_hull_quickfix
  //x = 2.450002; y = 2.038902; orientation = 1.5708; // after 15 iterations position to check not in supporting polygon TOO STEEP
  //x = 2.4; y = 1.55; orientation = 1.570796; // too low pos rating
  //x = 3.600002; y = 3.000002; orientation = 1.570796; // posrating = 0.0 should not be!
  x = 0.500002; y = 0.500002; orientation = 3.141593; // pos rating too low


  pcl::PointXYZ check_pos = pcl::PointXYZ(x, y, 0.0);
  //float orientation = (0.0)/180.0*3.14;
  float position_rating = 10.0;
  int unstable_axis = 10;
  pcl::PointXYZ pc, p0, p1, p2, p3;
#ifdef viewer_on
  terrain_model.computePositionRating(check_pos, orientation, pc, p0, p1, p2, p3,
                                            position_rating, unstable_axis, viewer, view_port_4);
#endif

  float xx=0.0;


  sensor_msgs::PointCloud2 cloud_point_msg;
  ROS_INFO("TerrainClassifierTest publishResult");
  if (cloud_input_pub.getNumSubscribers() >= 0)
  {
    ROS_INFO("TerrainClassifierNode publish cloud input");
    pcl::toROSMsg(*(terrain_classifier->getCloudInput()), cloud_point_msg);
    cloud_point_msg.header.stamp = ros::Time::now();
    cloud_point_msg.header.frame_id = terrain_classifier->getFrameId();
    cloud_input_pub.publish(cloud_point_msg);
  }
  bool subscribed = false;


  while (!viewer.wasStopped())
  {

      if (cloud_input_pub.getNumSubscribers() == 0)
      {
          cloud_input_pub.publish(cloud_point_msg);
      }
      else if(subscribed == false)
      {
          cloud_input_pub.publish(cloud_point_msg);
          subscribed = true;
      }
   //   viewer.removeAllPointClouds(4);
     // viewer.removeAllShapes(4);
      xx=xx+0.03;
    viewer.spinOnce(100);
    ros::spinOnce();
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "terrain_test");
  ros::Time::init();

  TerrainClassifierTest test;
  test.test_terrain_classifier();

  return 0;
}
