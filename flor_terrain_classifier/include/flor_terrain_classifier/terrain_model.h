#ifndef TERRAIN_MODEL_H__
#define TERRAIN_MODEL_H__

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
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

#include <nav_msgs/OccupancyGrid.h>

//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/geometry/planar_polygon.h>

//#include <pcl/surface/mls_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>


#include <iostream>
#include <pcl/io/pcd_io.h>


namespace hector_terrain_model
{

class TerrainModel
{
public:
    TerrainModel();
    TerrainModel(pcl::PointCloud<pcl::PointXYZ> cloud);
    virtual ~TerrainModel();

    bool findSupportPoint(const pcl::PointXYZ& tip_over_axis_point,
                          const pcl::PointXYZ& tip_over_axis_vector,
                          const pcl::PointCloud<pcl::PointXYZI> &pointcloud_robo,
                          const pcl::PointXYZ& tip_over_direction,
                          pcl::PointXYZ& support_point);

    std::vector<pcl::PointXYZ> buildConvexHull(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_positionRating,
                                               const pcl::PointXYZ& check_pos,
                                               const pcl::PointXYZ& support_point_1,
                                               const pcl::PointXYZ& support_point_2,
                                               const pcl::PointXYZ& support_point_3,
                                               const bool iterative,
                                               std::vector<unsigned int>& convex_hull_indices,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_positionRating2);

    void computeRobotPosition(const pcl::PointXYZ support_point_1, const pcl::PointXYZ support_point_2, const pcl::PointXYZ support_point_3,
                              const pcl::PointXYZ p0, const pcl::PointXYZ p1, const pcl::PointXYZ p2, const pcl::PointXYZ p3,
                              const Eigen::Vector3f& offset_CM,
                              pcl::PointXYZ& normal,
                              pcl::PointXYZ& robot_point_0, pcl::PointXYZ& robot_point_1, pcl::PointXYZ& robot_point_2, pcl::PointXYZ& robot_point_3,
                              pcl::PointXYZ& robot_point_mid, pcl::PointXYZ& robot_center_of_mass);

    pcl::PointXYZ computeCenterOfMass(const pcl::PointXYZ& p1_left,
                                      const pcl::PointXYZ& p2_left,
                                      const Eigen::Vector3f &normal,
                                      const pcl::PointXYZ &mid,
                                      const Eigen::Vector3f &offset);

    bool computePositionRating(const pcl::PointXYZ& checkPos,
                               const float orientation,
                               float position_rating,
                               int unstable_axis,
                               pcl::visualization::PCLVisualizer &viewer, int viewport);


    void showPositionRating(pcl::visualization::PCLVisualizer &viewer, const std::string &name = "positionRating", int viewport = 0) const;

    void updateCloud(pcl::PointCloud<pcl::PointXYZ> cloud);

    //pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_input;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_processed_Ptr;
    pcl::PointCloud<pcl::PointXYZ> cloud_processed;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outfiltered;
    // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_points_with_normals;
    //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_height;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_positionRating;
    pcl::PlanarPolygon<pcl::PointXYZ> supportingPolygon;


    static const float invalid_rating = 1.0;

private:
    float planeDistance(const pcl::PointXYZ& testpoint, const pcl::PointXYZ& plane_n, const pcl::PointXYZ& plane_p);
    bool atPlaneTest(const pcl::PointXYZ& testpoint, const pcl::PointXYZ& plane_n, const pcl::PointXYZ& plane_p, const float& delta);
    pcl::PointXYZ planeProjection(const pcl::PointXYZ& projection_p, const pcl::PointXYZ& plane_n, const pcl::PointXYZ& plane_p);

};
}

#endif
