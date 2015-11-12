#ifndef PCL_MATH_UTILS_H__
#define PCL_MATH_UTILS_H__

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>

#include <pcl/surface/vtk_smoothing/vtk.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

#include <nav_msgs/OccupancyGrid.h>

#include <pcl/geometry/planar_polygon.h>

//#include <pcl/surface/mls_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>


#include <iostream>
#include <pcl/io/pcd_io.h>


namespace hector_ground_contact_estimator
{

bool  pointsEqual(const pcl::PointXYZ& p1,const  pcl::PointXYZ& p2);

bool  pointsEqualxy(const pcl::PointXYZ& p1,const  pcl::PointXYZ& p2);

bool  pointsEqualxy(const pcl::PointXYZI& p1,const  pcl::PointXYZI& p2);

pcl::PointXYZ addPointVector(const pcl::PointXYZ& p1,const  pcl::PointXYZ& p2);

pcl::PointXYZ addPointVector(const pcl::PointXYZ& p1,const  Eigen::Vector3f& vector);

pcl::PointXYZ subtractPoints(const pcl::PointXYZ& p1,const pcl::PointXYZ& p2);

Eigen::Vector3f subtractPointsEigen(const pcl::PointXYZ& p1,const pcl::PointXYZ& p2);
// z stays
pcl::PointXYZ rotatePointZ(pcl::PointXYZ p, float degree /*radiants*/);

float dotproductEigen(Eigen::Vector3f v1, Eigen::Vector3f v2);

float dotProduct(const pcl::PointXYZ& a,const  pcl::PointXYZ& b);

pcl::PointXYZ crossProduct(const pcl::PointXYZ& a,const  pcl::PointXYZ& b);

Eigen::Vector3f crossProductEigen(const pcl::PointXYZ& a,const  pcl::PointXYZ& b);

// returns angle value in degree
float angleBetween(Eigen::Vector3f v1, Eigen::Vector3f v2);

// distance between 2 points (only seen in Z = 0 plane)
float distanceXY(const pcl::PointXYZ p1, const pcl::PointXYZ p2);

// distance between 2 points - 3D
float distanceXYZ(const pcl::PointXYZ p1, const pcl::PointXYZ p2);
// computes distance between straight / axis (ps, vector) and the point p

float distancePointStraight(pcl::PointXYZ ps, pcl::PointXYZ vector, pcl::PointXYZ p);
// 1 if positive, 0 if zero, -1 if negative

int sign(float a);
// return > 0 -> counterclockwise, // return < 0 -> clockwise  // return = 0 -> neither nor

float ccw(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3);

float angleToGround(const pcl::PointXYZ s1, const pcl::PointXYZ s2, const pcl::PointXYZ s3);

float planeDistance(const pcl::PointXYZ& testpoint, const pcl::PointXYZ& plane_n, const pcl::PointXYZ& plane_p);

pcl::PointXYZ planeProjection(const pcl::PointXYZ& projection_p, const pcl::PointXYZ& plane_n, const pcl::PointXYZ& plane_p);

bool atPlaneTest(const pcl::PointXYZ& testpoint, const pcl::PointXYZ& plane_n, const pcl::PointXYZ& plane_p);
}

#endif
