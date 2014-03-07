#include <footstep_planner/terrain_model.h>

#include <pcl_conversions/pcl_conversions.h>

namespace footstep_planner
{
TerrainModel::TerrainModel(const flor_terrain_classifier::TerrainModel::ConstPtr& terrain_model, double res, const geometry_msgs::Vector3 &foot_size, unsigned int min_sampling_steps_x, unsigned int min_sampling_steps_y, unsigned int max_sampling_steps_x, unsigned int max_sampling_steps_y, double max_intrusion_z, double max_ground_clearance)
  : resolution(res)
  , height_map_scale(terrain_model->height_map_scale)
  , foot_size(foot_size)
  , min_sampling_steps_x(min_sampling_steps_x)
  , min_sampling_steps_y(min_sampling_steps_y)
  , max_sampling_steps_x(max_sampling_steps_x)
  , max_sampling_steps_y(max_sampling_steps_y)
  , max_intrusion_z(max_intrusion_z)
  , max_ground_clearance(max_ground_clearance)
{
  ground_level_map.reset(new nav_msgs::OccupancyGrid(terrain_model->ground_level_map));

  // generate kd-tree
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_points_with_normals(new pcl::PointCloud<pcl::PointNormal>());
  pcl::fromROSMsg(terrain_model->cloud_points_with_normals, *cloud_points_with_normals);
  points_with_normals_kdtree.reset(new pcl::KdTreeFLANN<pcl::PointNormal>());
  points_with_normals_kdtree->setInputCloud(cloud_points_with_normals);

  height_map.reset(new nav_msgs::OccupancyGrid(terrain_model->height_map));
}

bool TerrainModel::getPointWithNormal(const pcl::PointNormal &p_search, pcl::PointNormal &p_result) const
{
  if (!points_with_normals_kdtree || points_with_normals_kdtree->getInputCloud()->size() == 0)
    return false;

  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;

  if (points_with_normals_kdtree->nearestKSearch(p_search, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
  {
    const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud = points_with_normals_kdtree->getInputCloud();

    for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
    {
      p_result = cloud->points[pointIdxNKNSearch[i]];
//      ROS_INFO("%f %f %f - sq_dist: %f", cloud->points[pointIdxNKNSearch[i]].x,
//                                         cloud->points[pointIdxNKNSearch[i]].y,
//                                         cloud->points[pointIdxNKNSearch[i]].z,
//                                         pointNKNSquaredDistance[i]);
    }

    if (pointNKNSquaredDistance[0] > resolution*resolution*1.1)
      return false;

    p_result = cloud->points[pointIdxNKNSearch[0]];
    return true;
  }

  return false;
}

bool TerrainModel::getHeight(double x, double y, double &height) const
{
  if (!height_map)
    return false;

  int map_x, map_y;
  toMapCoordinates(*height_map, x, y, map_x, map_y);

  // check if point is in map
  if (map_x < 0 || height_map->info.width <= (unsigned int)map_x || map_y < 0 || height_map->info.height <= (unsigned int)map_y)
    return false;

  // get height and rescale
  height = height_map->info.origin.position.z + (height_map->data.at(map_x + map_y * height_map->info.width) + 127) * height_map_scale;

  return true;
}

bool TerrainModel::getFootContactSupport(const State& s, double &support, pcl::PointCloud<pcl::PointXYZI>::Ptr checked_positions) const
{
  if (!height_map)
    return false;

  if (!getFootContactSupport(s, support, min_sampling_steps_x, min_sampling_steps_y, checked_positions))
    return false;

  // refinement of solution if needed
  if (support != 1.0)
  {
    if (!getFootContactSupport(s, support, max_sampling_steps_x, max_sampling_steps_y, checked_positions))
      return false;
  }

  return true;
}

bool TerrainModel::getFootContactSupport(const State& s, double &support, unsigned int sampling_steps_x, unsigned int sampling_steps_y, pcl::PointCloud<pcl::PointXYZI>::Ptr checked_positions) const
{
  support = 0.0;

  if (!height_map)
    return false;

  unsigned int contacts = 0;
  unsigned int total = 0;

  tf::Vector3 orig_pos;
  orig_pos.setZ(0.0);

  double foot_size_half_x = 0.5*foot_size.x;
  double foot_size_half_y = 0.5*foot_size.y;

  double sampling_step_x = foot_size.x/(double)(sampling_steps_x-1);
  double sampling_step_y = foot_size.y/(double)(sampling_steps_y-1);

  for (double y = -foot_size_half_y; y <= foot_size_half_y; y+=sampling_step_y)
  {
    orig_pos.setY(y);
    for (double x = -foot_size_half_x; x <= foot_size_half_x; x+=sampling_step_x)
    {
      total++;

      // determine point in world frame and get height at this point
      orig_pos.setX(x);

      const tf::Vector3 &trans_pos = s.getPose() * orig_pos;

      double height = 0.0;
      if (!getHeight(trans_pos.getX(), trans_pos.getY(), height))
      {
        ROS_WARN_THROTTLE(1.0, "getFootSupportArea: No height data found at %f/%f", s.getX(), s.getY());
        continue;
      }

      // diff heights
      double diff = trans_pos.getZ()-height;

      // save evaluated point for visualization
      if (checked_positions)
      {
        pcl::PointXYZI p;
        p.x = trans_pos.getX();
        p.y = trans_pos.getY();
        p.z = trans_pos.getZ();
        p.intensity = std::abs(diff);
        checked_positions->push_back(p);

        //ROS_INFO("%f %f | %f %f | %f", x, y, p.z, height, diff);
      }

      // check diff in z
      if (diff < -max_intrusion_z) // collision -> no support!
        return true;
//      else if (ivGroundLevelMapPtr->isOccupiedAt(trans_pos.getX(), trans_pos.getY()))
//        continue;
      else if (diff < max_ground_clearance) // ground contact
        contacts++;
    }
  }

  /// @ TODO: refinement (center of pressure)
  support = (double)contacts/(double)total;
  return true;
}

bool TerrainModel::add3DData(State &s) const
{
  double z = s.getZ();
  s.setGroundContactSupport(0.0);

  // get z
  if (!getHeight(s.getX(), s.getY(), z))
  {
    ROS_WARN_THROTTLE(1.0, "No height data found at %f/%f", s.getX(), s.getY());
    return false;
  }
  s.setZ(z);

  // get roll and pitch
  pcl::PointNormal p;
  p.x = s.getX();
  p.y = s.getY();
  p.z = s.getZ();

  if (!getPointWithNormal(p, p))
  {
    ROS_WARN_THROTTLE(1.0, "No normal data found at %f/%f", s.getX(), s.getY());
    return false;
  }
  s.setNormal(p.normal_x, p.normal_y, p.normal_z);

  // determine ground contact support
  double support = 0.0;
  if (!getFootContactSupport(s, support))
  {
    ROS_WARN_THROTTLE(1.0, "Couldn't determine ground contact support at %f/%f", s.getX(), s.getY());
    return false;
  }
  s.setGroundContactSupport(support);

  return true;
}

void TerrainModel::toMapCoordinates(const nav_msgs::OccupancyGrid &map, double x, double y, int &map_x, int &map_y) const
{
  map_x = floor((x-map.info.origin.position.x)/map.info.resolution);
  map_y = floor((y-map.info.origin.position.y)/map.info.resolution);
}
}
