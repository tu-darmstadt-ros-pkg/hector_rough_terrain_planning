#ifndef TERRAIN_MODEL_H__
#define TERRAIN_MODEL_H__

#define viewer_on

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/convex_hull.h>
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

#include <pcl/visualization/pcl_visualizer.h>
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
                          const pcl::PointCloud<pcl::PointXYZI> &pointcloud_robot,
                          const pcl::PointXYZ& tip_over_direction,
                          pcl::PointXYZ& support_point);

    std::vector<pcl::PointXYZ> buildConvexHull(const pcl::PointCloud<pcl::PointXYZI>::Ptr robot_pcl,
                                               const pcl::PointXYZ& check_pos,
                                               const pcl::PointXYZ& support_point_1,
                                               const pcl::PointXYZ& support_point_2,
                                               const pcl::PointXYZ& support_point_3,
                                               const bool iterative,
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

    void computeRobotCornerPoints(const pcl::PointXYZ& check_pos, float orientation,
                                pcl::PointXYZ& p0, pcl::PointXYZ& p1, pcl::PointXYZ& p2, pcl::PointXYZ& p3);

    void fillRobotPointcloud(const pcl::PointXYZ& p0, const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3,
                                           unsigned int& highest_Point_idx);

    bool computePositionRating(const pcl::PointXYZ& checkPos,
                               const float orientation,
                               pcl::PointXYZ &robot_point_center,
                               pcl::PointXYZ &robot_point_0, pcl::PointXYZ &robot_point_1, pcl::PointXYZ &robot_point_2, pcl::PointXYZ &robot_point_3, float &position_rating, int &unstable_axis
                                , pcl::visualization::PCLVisualizer* viewer = 0,
                               int viewport_1=1, int viewport_2=2, int viewport_3=3, int viewport_4=4, bool use_visualization=false);

    std::vector<float> computeForceAngleStabilityMetric(const pcl::PointXYZ& center_of_mass_pcl, std::vector<pcl::PointXYZ>& convex_hull_points_pcl);

    void updateCloud(pcl::PointCloud<pcl::PointXYZ> cloud);

    float optimalPossiblePositionRating();

    // Parameter initialized in contructor
    float robot_length;  // x
    float robot_width; // y
    Eigen::Vector3f offset_CM;
    float invalid_rating;
    float invalid_angle;
    float minimum_distance;
    float delta_for_contact;
    bool distance_smoothing;
    bool angle_smoothing;
    float smooth_max_angle;
    float smooth_max_distance;
    bool first_SP_around_mid;
    bool tip_over;
    bool check_each_axis;


    bool draw_convex_hull_first_polygon;
    bool draw_convex_hull_ground_points;
    bool draw_convex_hull_iterative;
    bool draw_convex_hull_iterative_ground_points;



    //pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_input;
    pcl::PointCloud<pcl::PointXYZ>::Ptr world_pcl_ptr;
    pcl::PointCloud<pcl::PointXYZ> cloud_processed;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outfiltered;
    //pcl::PointCloud<pcl::PointNormal>::Ptr cloud_points_with_normals;
    //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_height;
    pcl::PointCloud<pcl::PointXYZI>::Ptr robot_pcl;
    pcl::PlanarPolygon<pcl::PointXYZ> supportingPolygon;


  //  static const float invalid_rating = 0.1; // actually 0
  //  static const float invalid_angle = 40; // degree


private:
    bool atPlaneTest(const pcl::PointXYZ& testpoint, const pcl::PointXYZ& plane_n, const pcl::PointXYZ& plane_p, const float& delta);

};
}

#endif