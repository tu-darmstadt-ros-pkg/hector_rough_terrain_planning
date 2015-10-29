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

#ifndef terrain_preprocessor_NODE_H__
#define terrain_preprocessor_NODE_H__

#include <ros/ros.h>
#include <tf/tf.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <hector_terrain_preprocessor/TerrainModelRequest.h>
#include <hector_terrain_preprocessor/TerrainModelService.h>
#include <hector_terrain_preprocessor/TerrainModel.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <hector_terrain_preprocessor/terrain_preprocessor.h>

namespace hector_terrain_preprocessor
{
class TerrainPreprocessorNode
{
public:
  TerrainPreprocessorNode();
  virtual ~TerrainPreprocessorNode();

  void loadTestPointCloud();

protected:
  bool terrainModelService(TerrainModelService::Request &req, TerrainModelService::Response &resp);

  void setPointCloud(const sensor_msgs::PointCloud2 &cloud_input);

  void generateTerrainModel(const TerrainModelRequest &req);
  bool generateTerrainModel();

  void publishResult() const;

  // subscribers
  ros::Subscriber point_cloud_sub;
  ros::Subscriber generate_terrain_model_sub;

  // service clients
  ros::ServiceClient point_cloud_client;

  ros::ServiceServer test_service;

  // publisher
  ros::Publisher cloud_input_pub;
  ros::Publisher cloud_points_processed_pub;
  ros::Publisher cloud_points_processed_low_res_pub;
  ros::Publisher cloud_points_outfiltered_pub;
  ros::Publisher cloud_normals_pub;
  ros::Publisher cloud_gradients_pub;
  ros::Publisher ground_level_grid_map_pub;
  ros::Publisher height_grid_map_pub;
  ros::Publisher mesh_surface_pub;
  ros::Publisher terrain_model_pub;

  // services
  ros::ServiceServer generate_terrain_model_srv;

  TerrainPreprocessor::Ptr terrain_preprocessor;

  // parameters
  geometry_msgs::Point min_bounding_box;
  geometry_msgs::Point max_bounding_box;
  uint32_t aggregation_size;
};
}

#endif
