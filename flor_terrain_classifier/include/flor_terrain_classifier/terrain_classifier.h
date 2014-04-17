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

#ifndef TERRAIN_CLASSIFIER_H__
#define TERRAIN_CLASSIFIER_H__

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>

//#include <pcl/surface/mls_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>

#include <pcl/surface/vtk_smoothing/vtk.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
namespace flor_terrain_classifier
{
enum filterMask
{
  FILTER_NONE                   = 0,
  FILTER_PASS_THROUGH           = 1,
  FILTER_STATISTICAL_OUTLIER    = 2,
  FILTER_VOXEL_GRID             = 4,
  FILTER_MLS_SMOOTH             = 8,

  FILTER_ALL                    = 255
};

struct TerrainClassifierParams
{
  TerrainClassifierParams()
  {
  }

  TerrainClassifierParams(const ros::NodeHandle &nh)
  {
    readParams(nh);
  }

  void readParams(const ros::NodeHandle &nh)
  {
    nh.param("filter_mask", (int&)filter_mask, (int)FILTER_ALL);
    nh.param("threads", threads, 4);

    nh.param("pass_through/field_name", pt_field_name, std::string("z"));
    nh.param("pass_through/min", pt_min, -0.2);
    nh.param("pass_through/max", pt_max, 0.5);

    nh.param("voxel_grid/lx", vg_lx, 0.01);
    nh.param("voxel_grid/ly", vg_ly, 0.01);
    nh.param("voxel_grid/lz", vg_lz, 2.00);

    nh.param("mls_smoother/radius", ms_radius, 0.1);

    nh.param("statistical_outlier/radius", so_radius, 0.01);
    nh.param("statistical_outlier/k", so_k, 25);

    nh.param("normal_estimator/radius", ne_radius, 0.05);

    nh.param("gradient_estimator/thresh", ge_thresh, 0.5);

    nh.param("edge_detector/radius", ed_radius, 0.05);
    nh.param("edge_detector/max_std", ed_max_std, 0.02);
    nh.param("edge_detector/non_max_supp_radius", ed_non_max_supp_radius, 0.05);

    nh.param("grid_map_generator/resolution", gg_res, 0.02);
    nh.param("grid_map_generator/reconstruct", gg_reconstruct, false);

    nh.param("misc/low_res", low_res, 0.05);
  }

  unsigned int filter_mask;

  // number of threads should be used
  int threads;

  // params for pass through filter
  std::string pt_field_name;
  double pt_min, pt_max;

  // params for voxel grid filter
  double vg_lx, vg_ly, vg_lz;

  // params for moving least squares smoother
  double ms_radius;

  // params for statistical outlier filter
  double so_radius;
  int so_k;

  // params for normal estimator
  double ne_radius;

  // params for gradients estimator
  double ge_thresh;

  // params for edge detector
  double ed_radius;
  double ed_max_std;
  double ed_non_max_supp_radius;

  // params for grid map generator
  double gg_res;
  bool gg_reconstruct;

  // misc params
  double low_res;
};

class TerrainClassifier
{
public:
  TerrainClassifier();
  TerrainClassifier(const TerrainClassifierParams &params);
  virtual ~TerrainClassifier();

  void setParams(const TerrainClassifierParams &params);
  void getParams(TerrainClassifierParams &params) const { params = this->params; }

  void addCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &inputcloud, const std::string &frame_id = "/world");

  // visulization helper
  void showNormals(pcl::visualization::PCLVisualizer &viewer, const std::string &name = "normals", int viewport = 0) const;
  void showGradients(pcl::visualization::PCLVisualizer &viewer, const std::string &name = "gradients", int viewport = 0) const;
  void showHeightDiff(pcl::visualization::PCLVisualizer &viewer, const std::string &name = "hightDiff", int viewport = 0) const;
  void showPositionRating(pcl::visualization::PCLVisualizer &viewer, const std::string &name = "positionRating", int viewport = 0) const;


  // some getters
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &getCloudInput() const;
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &getCloudProcessed() const;
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &getCloudOutfiltered() const;
  const pcl::PointCloud<pcl::PointNormal>::Ptr &getPointsWithsNormals() const;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr &getGradients() const;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr &getEdges() const;

  void getCloudProcessedLowRes(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const;

  const nav_msgs::OccupancyGrid::Ptr &getGroundLevelGridMap() const;
  const nav_msgs::OccupancyGrid::Ptr &getHeightGridMap(float &height_map_scale) const;
  const nav_msgs::OccupancyGrid::Ptr &getHeightGridMap() const;

  const pcl::PolygonMesh::Ptr &getMeshSurface() const;

  const std::string &getFrameId() const { return frame_id; }

  // data generation
  bool computeNormals();
  bool computeGradients();
  bool computeHeightRating();
  bool generateGroundLevelGridmap();
  bool generateHeightGridmap();
  bool computeSurfaceMesh();



  pcl::PointXYZ eval_point(const pcl::PointXYZ& tip_over_axis_point,
                                              const pcl::PointXYZ& tip_over_axis_vector,
                                              const pcl::PointCloud<pcl::PointXYZI> &pointcloud_robo,
                                              const pcl::PointXYZ& tip_over_direction);
  bool computePositionRating(const pcl::PointXYZ& checkPos, pcl::visualization::PCLVisualizer &viewer, const std::string &name, int viewport);

  pcl::PointXYZ lastRatedPosition;

  // typedefs
  typedef boost::shared_ptr<TerrainClassifier> Ptr;
  typedef boost::shared_ptr<const TerrainClassifier> ConstPtr;

protected:
  void setDataOutdated();

  // filter
  template <typename PointT> void filterPassThrough(typename pcl::PointCloud<PointT>::Ptr &cloud, const std::string &field_name, double min, double max) const
  {
    if (!cloud)
    {
      ROS_ERROR("filterPassThrough was called but no point cloud was available");
      return;
    }

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(field_name);
    pass.setFilterLimits(min, max);
    pass.filter(*cloud_filtered);
    cloud = cloud_filtered;
  }

  template <typename PointT> void filterVoxelGrid(typename pcl::PointCloud<PointT>::Ptr &cloud, float lx, float ly, float lz) const
  {
    if (!cloud)
    {
      ROS_ERROR("filterVoxelGrid was called but no point cloud was available");
      return;
    }

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(lx, ly, lz);
    vox.filter(*cloud_filtered);
    cloud = cloud_filtered;
  }

  template <typename PointT> void filterMlsSmooth(typename pcl::PointCloud<PointT>::Ptr &cloud, double radius) const
  {
    if (!cloud)
    {
      ROS_ERROR("filterSmooth was called but no point cloud was available");
      return;
    }

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    pcl::MovingLeastSquares<PointT, PointT> mls;
    mls.setInputCloud(cloud);
    mls.setSearchMethod(tree);
    //mls.setComputeNormals(false);
    //mls.setPolynomialFit(false);
    mls.setPolynomialOrder(1);
    mls.setSearchRadius(radius);
  //  mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::SAMPLE_LOCAL_PLANE);
  //  mls.setUpsamplingRadius(0.025);
  //  mls.setUpsamplingStepSize(0.01);
  //  mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::RANDOM_UNIFORM_DENSITY);
  //  mls.setPointDensity(radius*5000);
  //  mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::VOXEL_GRID_DILATION);
  //  mls.setDilationVoxelSize(0.01);
  //  mls.setDilationIterations(1);
    mls.process(*cloud_filtered);
    cloud = cloud_filtered;
  }

  template <typename PointT> void filterStatisticalOutlier(typename pcl::PointCloud<PointT>::Ptr &cloud, int k, double radius, bool set_negative = false) const
  {
    if (!cloud)
    {
      ROS_ERROR("filterStatisticalOutlier was called but no point cloud was available");
      return;
    }

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setNegative(set_negative);
    sor.setMeanK(k);
    sor.setStddevMulThresh(radius);
    sor.filter(*cloud_filtered);
    cloud = cloud_filtered;
  }

  // some helper
  void getGridMapIndex(const nav_msgs::OccupancyGrid::ConstPtr &map, float x, float y, int &idx) const;
  void getGridMapCoords(const nav_msgs::OccupancyGrid::ConstPtr &map, unsigned int idx, float &x, float &y) const;

  float planeDistance(const pcl::PointXYZ& testpoint, const pcl::PointXYZ& plane_n, const pcl::PointXYZ& plane_p);
  bool atPlaneTest(const pcl::PointXYZ& testpoint, const pcl::PointXYZ& plane_n, const pcl::PointXYZ& plane_p, const float& delta);
  pcl::PointXYZ planeProjection(const pcl::PointXYZ& projection_p, const pcl::PointXYZ& plane_n, const pcl::PointXYZ& plane_p);

  TerrainClassifierParams params;

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_input;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_processed;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outfiltered;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_points_with_normals;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_gradients;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_edges;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_positionRating;

  nav_msgs::OccupancyGrid::Ptr ground_level_grid_map;
  nav_msgs::OccupancyGrid::Ptr height_grid_map;

  pcl::PolygonMesh::Ptr mesh_surface;

  pcl::PlanarPolygon<pcl::PointXYZ> supportingPolygon;


  std::string frame_id;

  double height_map_scale;

  bool cloud_normals_outdated;
  bool cloud_gradients_outdated;
  bool cloud_edges_outdated;

  bool ground_level_grid_map_outdated;
  bool height_grid_grid_map_outdated;

  bool mesh_surface_outdated;

  bool lock_input_cloud;
};
}

#endif
