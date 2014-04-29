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

#include <pcl/io/pcd_io.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/intensity_gradient.h>

#include <flor_terrain_classifier/terrain_classifier.h>


using namespace flor_terrain_classifier;

void test_normals()
{
  ros::NodeHandle nh_("~");
  flor_terrain_classifier::TerrainClassifierParams params(nh_);
  params.filter_mask = FILTER_PASS_THROUGH | FILTER_VOXEL_GRID | FILTER_MLS_SMOOTH;
  flor_terrain_classifier::TerrainClassifier::Ptr terrain_classifier(new flor_terrain_classifier::TerrainClassifier(params));

  ROS_INFO("Load point cloud");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::io::loadPCDFile("../pointclouds/ramp2_filtered.pcd", *cloud_original);
  //pcl::io::loadPCDFile("../pointclouds/konststeigend_x.pcd", *cloud_original);
  pcl::io::loadPCDFile("../pointclouds/zwei_ebenen_steigend.pcd", *cloud_original);
 // pcl::io::loadPCDFile("../pointclouds/dach.pcd", *cloud_original);


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

  // Position, Orientation (Wie genau?)
  terrain_classifier->computePositionRating(pcl::PointXYZ(-0.55,-1.05,0.5), 3.14/2,viewer, "computePositionRating", view_port_4);//1.00,-0.85,0
  //check 1.50,-1.35,0)
  terrain_classifier->showPositionRating(viewer, "positionRating", view_port_4);
  //viewer.addPointCloud<pcl::PointXYZ>(terrain_classifier->getCloudInput(), "input cloud", view_port_4);

  float x=0.0;

  while (!viewer.wasStopped())
  {
   //   viewer.removeAllPointClouds(4);
     // viewer.removeAllShapes(4);
      x=x+0.03;
    viewer.spinOnce(100);
    ros::spinOnce();
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
   // terrain_classifier->computePositionRating(pcl::PointXYZ(-1.00+x,-1.00,0.5),viewer, "computePositionRating", view_port_4);//1.00,-0.85,0

   // terrain_classifier->showPositionRating(viewer, "positionRating", view_port_4);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "terrain_test");
  ros::Time::init();

  test_normals();

  return 0;
}
