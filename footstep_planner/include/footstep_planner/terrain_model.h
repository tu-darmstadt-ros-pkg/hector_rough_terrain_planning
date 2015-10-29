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

#ifndef TERRAIN_MODEL_H
#define TERRAIN_MODEL_H

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <flor_terrain_classifier/TerrainModel.h>

#include <footstep_planner/State.h>

namespace footstep_planner
{
class TerrainModel
{
public:
  TerrainModel(const flor_terrain_classifier::TerrainModel::ConstPtr& terrain_model, double res, const geometry_msgs::Vector3 &foot_size, unsigned int min_sampling_steps_x, unsigned int min_sampling_steps_y, unsigned int max_sampling_steps_x, unsigned int max_sampling_steps_y, double max_intrusion_z, double max_ground_clearance);

  double getResolution() const { return resolution; }

  bool getPointWithNormal(const pcl::PointNormal &p_search, pcl::PointNormal &p_result) const;
  bool getHeight(double x, double y, double &height) const;
  bool getFootContactSupport(const State& s, double &support, pcl::PointCloud<pcl::PointXYZI>::Ptr checked_positions = pcl::PointCloud<pcl::PointXYZI>::Ptr()) const;

  bool add3DData(State &s) const;

  // typedefs
  typedef boost::shared_ptr<TerrainModel> Ptr;
  typedef boost::shared_ptr<TerrainModel> ConstPtr;

protected:
  bool getFootContactSupport(const State& s, double &support, unsigned int sampling_steps_x, unsigned int sampling_steps_y, pcl::PointCloud<pcl::PointXYZI>::Ptr checked_positions = pcl::PointCloud<pcl::PointXYZI>::Ptr()) const;

  void toMapCoordinates(const nav_msgs::OccupancyGrid &map, double x, double y, int &map_x, int &map_y) const;

  nav_msgs::OccupancyGridConstPtr ground_level_map;

  /// kd-tree for cloud points with normals
  pcl::KdTreeFLANN<pcl::PointNormal>::Ptr points_with_normals_kdtree;
  const double resolution;

  nav_msgs::OccupancyGridConstPtr height_map;
  const double height_map_scale;

  // robot params
  const geometry_msgs::Vector3 &foot_size;

  const unsigned int min_sampling_steps_x;  // min number of sampling steps in y
  const unsigned int min_sampling_steps_y;  // min number of sampling steps in y
  const unsigned int max_sampling_steps_x;  // max number of sampling steps in y
  const unsigned int max_sampling_steps_y;  // max number of sampling steps in y
  const double max_intrusion_z;             // how deep the foot may intrude into other objects (z axis only)#
  const double max_ground_clearance;        // maximal distance before a point is treated as "in the air"
};
}

#endif
