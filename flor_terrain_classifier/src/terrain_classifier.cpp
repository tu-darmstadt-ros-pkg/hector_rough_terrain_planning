#include <flor_terrain_classifier/terrain_classifier.h>

namespace flor_terrain_classifier
{
TerrainClassifier::TerrainClassifier()
  : frame_id("/world")
  , lock_input_cloud(false)
{
  setDataOutdated();
}

TerrainClassifier::TerrainClassifier(const TerrainClassifierParams &params)
  : frame_id("/world")
  , lock_input_cloud(false)
{
  setParams(params);
}

TerrainClassifier::~TerrainClassifier()
{
}

void TerrainClassifier::setParams(const TerrainClassifierParams &params)
{
  this->params = params;

  if (params.filter_mask & FILTER_PASS_THROUGH)           // cut out area of interest
    ROS_INFO("Using FILTER_PASS_THROUGH");
  if (params.filter_mask & FILTER_STATISTICAL_OUTLIER)    // remove outliers
    ROS_INFO("Using FILTER_STATISTICAL_OUTLIER");
  if (params.filter_mask & FILTER_VOXEL_GRID)             // summarize data
    ROS_INFO("Using FILTER_VOXEL_GRID");
  if (params.filter_mask & FILTER_MLS_SMOOTH)             // smooth data
    ROS_INFO("Using FILTER_MLS_SMOOTH");

  // clear old data
  if (cloud_input)
    cloud_processed.reset(new pcl::PointCloud<pcl::PointXYZ>(*cloud_input));
  setDataOutdated();
}

void TerrainClassifier::addCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, const std::string &frame_id)
{
  if (lock_input_cloud)
    return;

  this->cloud_input = cloud;
  this->frame_id = frame_id;

  // clear old data
  cloud_processed.reset(new pcl::PointCloud<pcl::PointXYZ>(*cloud_input));
  setDataOutdated();
}

void TerrainClassifier::showHeight(pcl::visualization::PCLVisualizer &viewer, const std::string &name, int viewport) const
{
  if (!cloud_height || cloud_height->empty())
  {
    ROS_WARN("showEdges was called before edges were detected!");
    return;
  }

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud_height, "intensity");
  viewer.addPointCloud<pcl::PointXYZI>(cloud_height, intensity_distribution, name + std::string("_cloud"), viewport);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name + std::string("_edges"), viewport);


}

const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &TerrainClassifier::getCloudInput() const
{
  if (!cloud_input || cloud_input->empty())
    ROS_WARN("getCloudInput was called before a input cloud was given!");
  return cloud_input;
}

const pcl::PointCloud<pcl::PointXYZ>::Ptr &TerrainClassifier::getCloudProcessed() const
{
  if (!cloud_processed || cloud_processed->empty())
    ROS_WARN("getCloudProcessed was called before a input cloud was given!");
  return cloud_processed;
}

const pcl::PointCloud<pcl::PointXYZ>::Ptr &TerrainClassifier::getCloudOutfiltered() const
{
  if (!cloud_outfiltered)
    ROS_WARN("getCloudOutfiltered was called before a input cloud was given!");
  return cloud_outfiltered;
}


void TerrainClassifier::getCloudProcessedLowRes(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const
{
  cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

  if (!cloud_points_with_normals || cloud_points_with_normals->empty())
    ROS_WARN("getCloudProcessedLow was called before normals were computed!");
  else
  {
    double half_low_res = 0.5*params.low_res;
    cloud->resize(cloud_points_with_normals->size());
    for (size_t i = 0; i < cloud->size(); i++)
    {
      cloud->at(i).x = cloud_points_with_normals->at(i).x;
      cloud->at(i).y = cloud_points_with_normals->at(i).y;
      cloud->at(i).z = cloud_points_with_normals->at(i).z - half_low_res; // reduce height so it matches vis with boxes
    }

    filterVoxelGrid<pcl::PointXYZ>(cloud,0.01,0.01,0.01);// params.low_res, params.low_res, 0.1);
  }
}



bool TerrainClassifier::computeNormals()
{
  // normals are up-to-date -> do nothing
  if (!cloud_normals_outdated && cloud_points_with_normals)
    return true;

  if (!cloud_input || cloud_input->empty())
  {
    ROS_ERROR("computeNormals was called but no point cloud was available");
    return false;
  }

  lock_input_cloud = true;

  // init map with standard data
  double ground_z = 0.0;
  //if (lower_body_state)
  //  ground_z = std::min(lower_body_state->left_foot_pose.position.z, lower_body_state->right_foot_pose.position.z);
  //else
    ROS_WARN("computeNormals: No state estimation of feet available. Assuming default ground level to be zero!");

  // preprocessing of cloud point
  cloud_processed.reset(new pcl::PointCloud<pcl::PointXYZ>(*cloud_input));
  if (params.filter_mask & FILTER_PASS_THROUGH)           // cut out area of interest
    filterPassThrough<pcl::PointXYZ>(cloud_processed, params.pt_field_name, ground_z + params.pt_min-0.3, ground_z + params.pt_max+0.3);
  if (params.filter_mask & FILTER_VOXEL_GRID)             // summarize data
    filterVoxelGrid<pcl::PointXYZ>(cloud_processed,0.03,0.03,0.03);// params.vg_lx, params.vg_ly, params.vg_lz);
  if (params.filter_mask & FILTER_MLS_SMOOTH)             // smooth data
    filterMlsSmooth<pcl::PointXYZ>(cloud_processed, params.ms_radius);

  if (cloud_processed->empty())
  {
    ROS_ERROR("Can compute normals. Got empty cloud point after filtering.");
    lock_input_cloud = false;
    return false;
  }

  // init normals data structure
  pcl::PointCloud<pcl::Normal> cloud_normals;
  cloud_points_with_normals.reset(new pcl::PointCloud<pcl::PointNormal>());

  // compute normals
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne(params.threads);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setInputCloud(cloud_processed);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(params.ne_radius);
  //ne.setKSearch(20);
  ne.compute(cloud_normals);

  pcl::concatenateFields(*cloud_processed, cloud_normals, *cloud_points_with_normals);

  // postprocessing
  if (params.filter_mask & FILTER_STATISTICAL_OUTLIER)    // remove outliers
  {
    cloud_outfiltered.reset(new pcl::PointCloud<pcl::PointXYZ>(*cloud_processed));
    filterStatisticalOutlier<pcl::PointNormal>(cloud_points_with_normals, params.so_k, params.so_radius);
    filterStatisticalOutlier<pcl::PointXYZ>(cloud_outfiltered, params.so_k, params.so_radius, true);
  }
  else
    cloud_outfiltered.reset(new pcl::PointCloud<pcl::PointXYZ>());

  // filter and some further postprocessing
  std::vector<int> indices;
  for (size_t i = 0; i < cloud_points_with_normals->size(); i++)
  {
    pcl::PointNormal &n = cloud_points_with_normals->at(i);
    if (!pcl_isfinite(n.normal_z))
      continue;

    indices.push_back((int)i);

    // flip all other normals in one direction
    if (n.normal_z < 0.0)
    {
      n.normal_x = -n.normal_x;
      n.normal_y = -n.normal_y;
      n.normal_z = -n.normal_z;
    }
  }

  // apply filtering
  pcl::PointCloud<pcl::PointNormal>::Ptr filtered(new pcl::PointCloud<pcl::PointNormal>());
  copyPointCloud(*cloud_points_with_normals, indices, *filtered);
  cloud_points_with_normals = filtered;

  cloud_normals_outdated = false;
  lock_input_cloud = false;
  return true;
}

bool TerrainClassifier::computeGradients()
{
  // gradients are up-to-date -> do nothing
  if (!cloud_gradients_outdated && cloud_gradients)
    return true;

  if (!computeNormals())
  {
    ROS_ERROR("Can't generate gradients!");
    return false;
  }

  lock_input_cloud = true;

  // init gradient data structure
  cloud_gradients.reset(new pcl::PointCloud<pcl::PointXYZI>());
  cloud_gradients->resize(cloud_points_with_normals->size());

  for (unsigned int i = 0; i < cloud_points_with_normals->size(); i++)
  {
    pcl::PointNormal &pn = cloud_points_with_normals->at(i);
    pcl::PointXYZI &pi = cloud_gradients->at(i);

    pi.x = pn.x;
    pi.y = pn.y;
    pi.z = pn.z;

    pi.intensity = sqrt(pn.normal_x*pn.normal_x + pn.normal_y*pn.normal_y); // = sqrt(1 - n.normal_z*n.normal_z)
    pi.intensity = pi.intensity < params.ge_thresh ? 1.0 : 0.0;
  }


  cloud_gradients_outdated = false;
  lock_input_cloud = false;
  return true;
}

bool TerrainClassifier::computeHeightRating()
{
  // edges are up-to-date -> do nothing
  if (!cloud_edges_outdated && cloud_height)
    return true;

  if (!computeNormals())
  {
    ROS_ERROR("Can't run edge detection!");
    return false;
  }

  lock_input_cloud = true;

  // init edge data structure
  cloud_height.reset(new pcl::PointCloud<pcl::PointXYZI>());
  cloud_height->resize(cloud_processed->size());

  // project all data to plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>());
  points->resize(cloud_processed->size());
  for (unsigned int i = 0; i < cloud_processed->size(); i++)
  {
    const pcl::PointXYZ &n = cloud_processed->at(i);
    pcl::PointXYZ &p = points->at(i);
    p.x = n.x;
    p.y = n.y;
    p.z = n.z;
  }


  for (size_t i = 0; i < points->size(); i++)
  {
    const pcl::PointXYZ &current = cloud_processed->at(i);
    pcl::PointXYZI &result = cloud_height->at(i);

    result.x = current.x;
    result.y = current.y;
    result.z = current.z;
    result.intensity = 0.0;

      if (result.z>0.5)
          result.intensity=0.5;
      else if (result.z<-0.1)
          result.intensity=-0.1;
      else
          result.intensity = result.z;




  }

  cloud_edges_outdated = false;
  lock_input_cloud = false;
  return true;
}

bool TerrainClassifier::generateGroundLevelGridmap()
{
  // generate new grid map only if needed
  if (!ground_level_grid_map_outdated && ground_level_grid_map)
    return true;

  if ((!cloud_gradients || !computeGradients()) && (!cloud_height || !computeHeightRating()))
  {
    ROS_ERROR("Can't generate grid map!");
    return false;
  }

  lock_input_cloud = true;

  // determine min and max coordinatesss
  double min_x, max_x;
  double min_y, max_y;

  min_x = min_y = FLT_MAX;
  max_x = max_y = FLT_MIN;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  if (cloud_gradients)
    cloud = cloud_gradients;
  else if (cloud_height)
    cloud = cloud_height;

  for (size_t i = 0; i < cloud->size(); i++)
  {
    const pcl::PointXYZI &p = cloud->at(i);
    min_x = std::min(min_x, (double)p.x);
    min_y = std::min(min_y, (double)p.y);
    max_x = std::max(max_x, (double)p.x);
    max_y = std::max(max_y, (double)p.y);
  }

  // setting up grid map
  ground_level_grid_map.reset(new nav_msgs::OccupancyGrid());

  ground_level_grid_map->info.resolution = params.gg_res;
  ground_level_grid_map->info.width = ceil((max_x-min_x) * 1/params.gg_res);
  ground_level_grid_map->info.height = ceil((max_y-min_y) * 1/params.gg_res);

  ground_level_grid_map->info.origin.position.x = min_x;
  ground_level_grid_map->info.origin.position.y = min_y;
  ground_level_grid_map->info.origin.position.z = 0.0;

  ground_level_grid_map->data.clear();
  ground_level_grid_map->data.resize(ground_level_grid_map->info.width * ground_level_grid_map->info.height, -1);

  // THIS IS A HACK, REMOVED IT SOON AS POSSIBLE
  bool ignore_near = false;
  double robot_x = 0.0;
  double robot_y = 0.0;
  //if (lower_body_state)
  //{
  //  robot_x = lower_body_state->pelvis_pose.position.x;
  //  robot_y = lower_body_state->pelvis_pose.position.y;
  //  ignore_near = true;
  //}

  // add data from gradients point cloud
  if (cloud_gradients)
  {
    ROS_INFO("...adding gradients");
    for (size_t i = 0; i < cloud_gradients->size(); i++)
    {
      const pcl::PointXYZI &p = cloud_gradients->at(i);

      if (ignore_near)
      {
        if ((p.x-robot_x)*(p.x-robot_x) + (p.y-robot_y)*(p.y-robot_y) < 0.4*0.4)
          continue;
      }

      int idx = -1;
      getGridMapIndex(ground_level_grid_map, p.x, p.y, idx);
      ground_level_grid_map->data.at(idx) = std::max(ground_level_grid_map->data.at(idx), (int8_t)floor((1.0-p.intensity) * 100));
    }
  }

  // add data from edge point cloud
  if (cloud_height)
  {
    ROS_INFO("...adding edges");
    ROS_ASSERT(cloud_gradients->size() == cloud_height->size());

    for (size_t i = 0; i < cloud_height->size(); i++)
    {
      const pcl::PointXYZI &p = cloud_height->at(i);

//      if (ignore_near)
//      {
//        if ((p.x-robot_x)*(p.x-robot_x) + (p.y-robot_y)*(p.y-robot_y) < 0.4*0.4)
//          continue;
//      }

      int idx = -1;
      getGridMapIndex(ground_level_grid_map, p.x, p.y, idx);
      ground_level_grid_map->data.at(idx) = std::max(ground_level_grid_map->data.at(idx), (int8_t)floor((1.0-p.intensity) * 100));
    }
  }

  ground_level_grid_map->header.stamp = ros::Time::now();
  ground_level_grid_map->header.frame_id = frame_id;

  ground_level_grid_map_outdated = false;
  lock_input_cloud = false;
  return true;
}

bool TerrainClassifier::generateHeightGridmap()
{
  // generate new grid map only if needed
  if (!height_grid_grid_map_outdated && height_grid_map)
    return true;

  if (!cloud_points_with_normals || !computeNormals())
  {
    ROS_ERROR("Can't generate grid map!");
    return false;
  }

  lock_input_cloud = true;

  double min_x, max_x;
  double min_y, max_y;
  double min_z, max_z;

  min_x = min_y = min_z = FLT_MAX;
  max_x = max_y = max_z = FLT_MIN;
  for (size_t i = 0; i < cloud_points_with_normals->size(); i++)
  {
    const pcl::PointNormal &p = cloud_points_with_normals->at(i);
    min_x = std::min(min_x, (double)p.x);
    min_y = std::min(min_y, (double)p.y);
    min_z = std::min(min_z, (double)p.z);
    max_x = std::max(max_x, (double)p.x);
    max_y = std::max(max_y, (double)p.y);
    max_z = std::max(max_z, (double)p.z);
  }

  double ground_z = 0.0;
  //if (lower_body_state)
  //  ground_z = std::min(lower_body_state->left_foot_pose.position.z, lower_body_state->right_foot_pose.position.z);
  //else
  //  ROS_WARN("generateHeightGridmap: No state estimation of feet available. Assuming default ground level to be zero!");

  min_z = std::min(min_z, ground_z);
  max_z = std::max(max_z, ground_z);

  // setting up grid map
  height_grid_map.reset(new nav_msgs::OccupancyGrid());

  height_grid_map->info.resolution = params.gg_res;
  height_grid_map->info.width = ceil((max_x-min_x) * 1/params.gg_res);
  height_grid_map->info.height = ceil((max_y-min_y) * 1/params.gg_res);

  height_grid_map->info.origin.position.x = min_x;
  height_grid_map->info.origin.position.y = min_y;
  height_grid_map->info.origin.position.z = min_z;

  height_grid_map->data.clear();
  height_grid_map->data.resize(height_grid_map->info.width * height_grid_map->info.height, -1);

  height_map_scale = (max_z-min_z)/254;
  double height_map_scale_inv = 1.0/height_map_scale;

  // init grid map
  for (size_t i = 0; i < height_grid_map->data.size(); i++)
    height_grid_map->data.at(i) = -128;

  // adding height information from cloud points
  for (size_t i = 0; i < cloud_points_with_normals->size(); i++)
  {
    const pcl::PointNormal &p = cloud_points_with_normals->at(i);

    int8_t height = (int8_t)floor((p.z-min_z) * height_map_scale_inv)-127;

    int idx = -1;
    getGridMapIndex(height_grid_map, p.x, p.y, idx);
    height_grid_map->data.at(idx) = std::max(height_grid_map->data.at(idx), height);
  }

  // fill missing data
  pcl::KdTreeFLANN<pcl::PointNormal> tree;
  tree.setInputCloud(cloud_points_with_normals);
  pcl::PointNormal current;
  current.z = ground_z;

  unsigned int k = 100;
  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;
  pcl::PointNormal result;

  pointIdxNKNSearch.resize(k);
  pointNKNSquaredDistance.resize(k);

  int8_t ground_level_height = (int8_t)floor((ground_z-min_z) * height_map_scale_inv)-127;

  for (size_t i = 0; i < height_grid_map->data.size(); i++)
  {
    int8_t &height = height_grid_map->data.at(i);
    if (height == -128)
    {
      if (params.gg_reconstruct)
      {
        getGridMapCoords(height_grid_map, i, current.x, current.y);

        // find k nearest neighbour and use their z
        if (tree.nearestKSearch(current, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
          double z = 0.0;
          float total_dist = 0.0;

          for (size_t n = 0; n < k; n++)
            total_dist += pointNKNSquaredDistance[n];

          for (size_t n = 0; n < k; n++)
          {
            result = cloud_points_with_normals->points[pointIdxNKNSearch[n]];
            z += result.z * (1-pointNKNSquaredDistance[n]/total_dist);
          }
          z /= k;

          height = (int8_t)floor((z-min_z) * height_map_scale_inv)-127;

          result.x = current.x;
          result.y = current.y;
          result.z = z;
          cloud_points_with_normals->push_back(result);
        }
      }
      else
        height = ground_level_height;
    }
  }

  height_grid_map->header.stamp = ros::Time::now();
  height_grid_map->header.frame_id = frame_id;

  height_grid_grid_map_outdated = false;
  lock_input_cloud = false;
  return true;
}

bool TerrainClassifier::computeSurfaceMesh()
{
  // mesh is up-to-date -> do nothing
  if (!mesh_surface_outdated && mesh_surface)
    return true;

  if (!computeNormals())
  {
    ROS_ERROR("Can't run surface reconstruction!");
    return false;
  }

  lock_input_cloud = true;

  // init mesh data structure
  mesh_surface.reset(new pcl::PolygonMesh());

  // Create search tree
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
  tree->setInputCloud(cloud_points_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius(0.10);

  // Set typical values for the parameters
  gp3.setMu(6.0);
  gp3.setMaximumNearestNeighbors(50);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud(cloud_points_with_normals);
  gp3.setSearchMethod(tree);
  gp3.reconstruct(*mesh_surface);

  // Additional vertex information
//  std::vector<int> parts = gp3.getPartIDs();
//  std::vector<int> states = gp3.getPointStates();

  mesh_surface_outdated = false;
  lock_input_cloud = false;
  return true;
}

void TerrainClassifier::setDataOutdated()
{
  cloud_normals_outdated = true;
  cloud_gradients_outdated = true;
  cloud_edges_outdated = true;
  ground_level_grid_map_outdated = true;
  height_grid_grid_map_outdated = true;
  mesh_surface_outdated = true;
}

void TerrainClassifier::getGridMapIndex(const nav_msgs::OccupancyGrid::ConstPtr &map, float x, float y, int &idx) const
{
  int map_x = floor((x-map->info.origin.position.x) * 1/map->info.resolution);
  int map_y = floor((y-map->info.origin.position.y) * 1/map->info.resolution);

  if (map_x < 0 || map->info.width <= (unsigned int)map_x || map_y < 0 || map->info.height <= (unsigned int)map_y)
    idx = -1;
  else
    idx = map_x + map_y * map->info.width;
}

void TerrainClassifier::getGridMapCoords(const nav_msgs::OccupancyGrid::ConstPtr &map, unsigned int idx, float &x, float &y) const
{
  unsigned int map_x = idx % map->info.width;
  unsigned int map_y = std::floor(idx / map->info.width);

  x = (float)map_x * map->info.resolution + map->info.origin.position.x;
  y = (float)map_y * map->info.resolution + map->info.origin.position.y;
}



float dotProduct(const pcl::PointXYZ& a,const  pcl::PointXYZ& b)
{
    return a.x*b.x+a.y*b.y+a.z*b.z;
}

pcl::PointXYZ crossProduct(const pcl::PointXYZ& a,const  pcl::PointXYZ& b)
{
    float cx=a.y*b.z-a.z*b.y;
    float cy=a.z*b.x-a.x*b.z;
    float cz=a.x*b.y-a.y*b.x;
    return pcl::PointXYZ(cx,cy,cz);
}
float TerrainClassifier::planeDistance(const pcl::PointXYZ& testpoint, const pcl::PointXYZ& plane_n, const pcl::PointXYZ& plane_p)
{
    float d=dotProduct(pcl::PointXYZ(testpoint.x-plane_p.x, testpoint.y-plane_p.y, testpoint.z-plane_p.z),plane_n)/sqrt(plane_n.x*plane_n.x+plane_n.y*plane_n.y+plane_n.z*plane_n.z);
    return d;
}

pcl::PointXYZ TerrainClassifier::planeProjection(const pcl::PointXYZ& projection_p, const pcl::PointXYZ& plane_n, const pcl::PointXYZ& plane_p)
{


    Eigen::Vector3f x = Eigen::Vector3f(projection_p.x,projection_p.y,projection_p.z);
    Eigen::Vector3f n = Eigen::Vector3f(plane_n.x,plane_n.y,plane_n.z);
    Eigen::Vector3f r = Eigen::Vector3f(plane_p.x,plane_p.y,plane_p.z);
    Eigen::Vector3f res = x - ((x-r).dot(n)/n.dot(n))*n;
    pcl::PointXYZ ret = pcl::PointXYZ(res[0],res[1],res[2]);
    return ret;
}



bool TerrainClassifier::atPlaneTest(const pcl::PointXYZ& testpoint, const pcl::PointXYZ& plane_n, const pcl::PointXYZ& plane_p, const float& delta)
{
    return (abs(planeDistance(testpoint,plane_n,plane_p))<=delta);
}


float ccw(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3)
{
    return (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
}


//convex hull computation
//  http://en.wikipedia.org/wiki/Gift_wrapping_algorithm
void convex_hull_comp(pcl::PointCloud<pcl::PointXYZ>& cloud,std::vector<unsigned int>& convex_hull_indices)
{
    float x_min=cloud.at(0).x;
    unsigned int point_on_hull=0;
    pcl::PointCloud<pcl::PointXYZ> cloud_2d;
    cloud_2d.resize(0);

    //find minx
    for (unsigned int i = 0; i < cloud.size(); i++)
    {
        pcl::PointXYZ p = cloud.at(i);
        cloud_2d.push_back(pcl::PointXYZ(p.x,p.y,0.0));
        if (p.x<x_min)
        {
            x_min=p.x;
            point_on_hull=i;
        }

    }
    //build hull
    int i=0;
    unsigned int endpoint;
    while (true)
    {
        convex_hull_indices.push_back(point_on_hull);
        endpoint=0;
        if((0==i)&&(0==point_on_hull))endpoint=1;
        for(unsigned int j=1; j<cloud_2d.size();++j)
        {
            pcl::PointXYZ& p0 =cloud_2d.at(convex_hull_indices.at(i));
            pcl::PointXYZ& p1 =cloud_2d.at(endpoint);
            pcl::PointXYZ& p2 =cloud_2d.at(j);
            float ccw_f=ccw(cloud_2d.at(convex_hull_indices.at(i)),cloud_2d.at(endpoint),cloud_2d.at(j));
            float dist_old=(p0.x-p1.x)*(p0.x-p1.x)+(p0.y-p1.y)*(p0.y-p1.y);
            float dist_new=(p0.x-p2.x)*(p0.x-p2.x)+(p0.y-p2.y)*(p0.y-p2.y);
            bool isleft=(ccw_f <0);
            bool isfurther=(ccw_f<=0.000001 && dist_new>dist_old);
            if(endpoint==point_on_hull || isleft || isfurther)
            {
                endpoint=j;
            }
        }

        i++;
        point_on_hull=endpoint;
        if(endpoint==convex_hull_indices.at(0))
        {
            convex_hull_indices.push_back(convex_hull_indices.at(0));
            break;
        }

    }
}
//As proposed in "Modeling the manipulator and flipper pose effects on tip over stability of a tracked mobile manipulator" by Chioniso Dube

std::vector<float> computeForceAngleStabilityMetric(const pcl::PointXYZ& center_of_mass_pcl, std::vector<pcl::PointXYZ>& convex_hull_points_pcl)
{
    const Eigen::Vector3f center_of_mass = Eigen::Vector3f(center_of_mass_pcl.x,center_of_mass_pcl.y,center_of_mass_pcl.z);
    std::vector<Eigen::Vector3f> p;
    Eigen::Vector3f f_r= Eigen::Vector3f(0.0,0.0,-10.0);

    for(unsigned int i=0; i<convex_hull_points_pcl.size();++i)
    {
         pcl::PointXYZ& pi = convex_hull_points_pcl.at(i);
         p.push_back(Eigen::Vector3f(pi.x,pi.y,pi.z));
    }
    std::vector<float> res;
    for(unsigned int i=0; i<(p.size()-1);++i)
    {
        Eigen::Vector3f p_i1;
        if(p.size()==(i+1)) p_i1 = p.at(0);
        else p_i1 = p.at(i+1);
         Eigen::Vector3f ai=p_i1-p.at(i);
         ai.normalize();
         Eigen::Matrix3f ident_mat =Eigen::Matrix3f::Identity();
         Eigen::Vector3f li=(ident_mat-ai*ai.transpose())*(p_i1-center_of_mass);
         Eigen::Vector3f fi=(ident_mat-ai*ai.transpose())*f_r;
         Eigen::Vector3f li_norm=li;
         Eigen::Vector3f fi_norm=fi;
         li_norm.normalize();
         fi_norm.normalize();
         Eigen::Vector3f di= (-li)+(li.dot(fi_norm))*fi_norm;
         float theta_i=acos(li_norm.dot(fi_norm));
         float sigma_i=((fi_norm.cross(li_norm)).dot(ai))?1.0:-1.0 ;


         float beta=sigma_i*theta_i*di.norm()*f_r.norm();
         res.push_back(beta);
    }

    return res;
}



// Point1 - Point2
pcl::PointXYZ subtractPoints(pcl::PointXYZ p1, pcl::PointXYZ p2){
    return pcl::PointXYZ(p1.x - p2.x, p1.y - p1.y, p1.z - p2.z);
}



// evaluates the second or third point of the supporting plane
pcl::PointXYZ TerrainClassifier::eval_point(const pcl::PointXYZ& tip_over_axis_point,
                                            const pcl::PointXYZ& tip_over_axis_vector,
                                            const pcl::PointCloud<pcl::PointXYZI>& pointcloud_robo,
                                            const pcl::PointXYZ& tip_over_direction)
{
    // project Cloud on plane with normal = tipoveraxis
    // tip_over_direction = pcl::PointXYZ(tip_over_direction.x, tip_over_direction.y, 0);

    pcl::PointCloud<pcl::PointXYZ> cloud_projected;
    cloud_projected.resize(pointcloud_robo.size());
    for(unsigned int i=0; i<cloud_projected.size();++i)
    {
        pcl::PointXYZ &p_pro= cloud_projected.at(i);
        const pcl::PointXYZI &p_pos= pointcloud_robo.at(i);
        p_pro.x=p_pos.x;
        p_pro.y=p_pos.y;
        p_pro.z=p_pos.z;
        const  pcl::PointXYZ &p=p_pro;
        cloud_projected.at(i)=planeProjection(p,tip_over_axis_vector,tip_over_axis_point);
    }

    //find supppoint
    float min_angle=360.0;  // Hier den angle von 5 auf 360 geändert, da der winkel auch generell größer sein kann.
    int min_angle_idx;
    const pcl::PointXYZ v1 = tip_over_direction;
    std::vector<float> angles;

    for(unsigned int i=0; i<cloud_projected.size();++i)
    {
        pcl::PointXYZ &p_pro= cloud_projected.at(i);
        const pcl::PointXYZ v2 = pcl::PointXYZ(p_pro.x-tip_over_axis_point.x,p_pro.y-tip_over_axis_point.y,p_pro.z-tip_over_axis_point.z);
        float angle= acos( dotProduct(v1,v2)/sqrt(dotProduct(v1,v1)*dotProduct(v2,v2)));
        // neu
        const float pi = 3.14159;
        angle = angle * 360.0/(2.0*pi);
        if (angle > 180.0) {angle = 360.0 - angle;}
        //ende neu
        angles.push_back(angle);
        if (angle<min_angle)
        {
            min_angle=angle;
            min_angle_idx=i;
            ROS_INFO("newminangle : %f ",angle);

        }
    }

    //Support_point computed
    const pcl::PointXYZ support_point=pcl::PointXYZ(pointcloud_robo.at(min_angle_idx).x,pointcloud_robo.at(min_angle_idx).y,pointcloud_robo.at(min_angle_idx).z);
    return support_point;
}

bool TerrainClassifier::computePositionRating(const pcl::PointXYZ& check_pos, pcl::visualization::PCLVisualizer &viewer, const std::string &name, int viewport)
{
     lastRatedPosition=check_pos;
     pcl::PointXYZ center_of_mass(0.0,0.0,1.0); //center of mass relative to checkpos(x,y,?)
     float widthx=0.50;
     float lengthy=1.20;
     float alpha=3.14/2;
     center_of_mass.x=cos(alpha)* center_of_mass.x-sin(alpha)* center_of_mass.y;
     center_of_mass.y=sin(alpha)* center_of_mass.x+cos(alpha)* center_of_mass.y;
     cloud_positionRating.reset(new pcl::PointCloud<pcl::PointXYZI>());
     unsigned int highest_Point_idx;
     unsigned int n_counter=0;

     //filter relevant points and find max
     const float x_max=check_pos.x+cos(alpha)*widthx*0.5-sin(alpha)*lengthy*0.5;
     const float x_min=check_pos.x-cos(alpha)*widthx*0.5+sin(alpha)*lengthy*0.5;
     const float y_max=check_pos.y+sin(alpha)*widthx*0.5+cos(alpha)*lengthy*0.5;
     const float y_min=check_pos.y-sin(alpha)*widthx*0.5-cos(alpha)*lengthy*0.5;
     const pcl::PointXYZ p0=pcl::PointXYZ(x_min,y_min,0);
     const pcl::PointXYZ p1=pcl::PointXYZ(x_max,y_min,0);
     const pcl::PointXYZ p2=pcl::PointXYZ(x_max,y_max,0);
     const pcl::PointXYZ p3=pcl::PointXYZ(x_min,y_max,0);


     bool hull_cpp= (ccw(p0,p1,p2)<0);
     for (unsigned int i = 0; i < cloud_processed->size(); i++)
     {
       const  pcl::PointXYZ &pp= cloud_processed->at(i);

       bool c0=hull_cpp ? (ccw(p0,p1,pp)<0) : (ccw(p0,p1,pp)>0);
       bool c1=hull_cpp ? (ccw(p1,p2,pp)<0) : (ccw(p1,p2,pp)>0);
       bool c2=hull_cpp ? (ccw(p2,p3,pp)<0) : (ccw(p2,p3,pp)>0);
       bool c3=hull_cpp ? (ccw(p3,p0,pp)<0) : (ccw(p3,p0,pp)>0);

       if(c0&&c1&&c2&&c3)
       {
           pcl::PointXYZI p= pcl::PointXYZI();
           p.x=pp.x;
           p.y=pp.y;
           p.z=pp.z;
           p.intensity=0.0;
           cloud_positionRating->push_back(p);

           if(n_counter==0) highest_Point_idx=1;
           else if(p.z>cloud_positionRating->at(highest_Point_idx).z) highest_Point_idx=n_counter;
           ++n_counter;
       }
     }

     if(cloud_positionRating->size()==0)
     {
         return false;
     }

     pcl::PointXYZI &p_max= cloud_positionRating->at(highest_Point_idx);
     int support_point_1_idx;
     float min_dist=-1.0;     
     for (unsigned int i = 0; i < cloud_positionRating->size(); i++)
     {
         pcl::PointXYZI& p = cloud_positionRating->at(i);
         if(((p.z-p_max.z)<0.00003)&&((p.z-p_max.z)>-0.00003))
         {
             float dist=sqrt((p.x-check_pos.x)*(p.x-check_pos.x)+(p.y-check_pos.y)*(p.y-check_pos.y));
             if(min_dist<0.0 || dist<min_dist)
             {
                 min_dist=dist;
                 support_point_1_idx=i;
                // ROS_INFO("i d : %i %f %f ",i,min_dist, dist);
             }

         }
     }
     const pcl::PointXYZ support_point_1 = pcl::PointXYZ(cloud_positionRating->at(support_point_1_idx).x,cloud_positionRating->at(support_point_1_idx).y,cloud_positionRating->at(support_point_1_idx).z);
     cloud_positionRating->at(support_point_1_idx).intensity=0.0;


     //find 2nd point of supp polygon

     const pcl::PointXYZ tip_over_axis_point = support_point_1;
     const pcl::PointXYZ tip_over_axis_vector = (crossProduct(pcl::PointXYZ(support_point_1.x-check_pos.x,support_point_1.y-check_pos.y,0),pcl::PointXYZ(0,0,1)));
     const pcl::PointXYZ tip_over_direction = pcl::PointXYZ(check_pos.x - support_point_1.x, check_pos.y - support_point_1.y, 0);

     const pcl::PointXYZ support_point_2 = eval_point(tip_over_axis_point,
                                                      tip_over_axis_vector,
                                                      (*cloud_positionRating),
                                                      tip_over_direction);

     viewer.addSphere(support_point_1, 0.02,1,0,0, "sp1", viewport);
     viewer.addSphere(support_point_2, 0.02,0,1,0, "sp2", viewport);

    //find third point
    const pcl::PointXYZ tip_over_axis_point_3 = support_point_1;
    const pcl::PointXYZ tip_over_axis_vector_3 = subtractPoints(support_point_2, support_point_1);

    // Fehler da auch in die andere Richtung definiert sein kann ich erst machen wenn das mit den richtungen überaupt
    // funktioniert.
    const pcl::PointXYZ tip_over_direction_3 = crossProduct(tip_over_axis_vector_3, pcl::PointXYZ(0,0,1));

    const pcl::PointXYZ support_point_3 = eval_point(tip_over_axis_point_3,
                                                      tip_over_axis_vector_3,
                                                      (*cloud_positionRating),
                                                      tip_over_direction_3);

 /*
     //neue ebene
     const pcl::PointXYZ p21= pcl::PointXYZ (support_point_2.x-support_point_1.x,support_point_2.y-support_point_1.y,support_point_2.z-support_point_1.z);
     const float e=dotProduct(p21,check_pos);
     const float e2=dotProduct(p21,support_point_1);
     const float e3=dotProduct(p21,p21);
     const float k=(e-e2)/e3;
     const pcl::PointXYZ lotFusPkt=pcl::PointXYZ(support_point_1.x+k*p21.x,support_point_1.y+k*p21.y,support_point_1.z+k*p21.z);

     //projection plane for support point 3
     const pcl::PointXYZ plane2_p = pcl::PointXYZ(lotFusPkt.x,lotFusPkt.y,lotFusPkt.z);
     const pcl::PointXYZ plane2_n = p21;

     pcl::PointCloud<pcl::PointXYZ> cloud_projected2;
     cloud_projected2.resize(cloud_positionRating->size());
     for(unsigned int i=0; i<cloud_projected2.size();++i)
     {
         pcl::PointXYZ &p_pro= cloud_projected2.at(i);
         pcl::PointXYZI &p_pos= cloud_positionRating->at(i);
         p_pro.x=p_pos.x;
         p_pro.y=p_pos.y;
         p_pro.z=p_pos.z;
         const  pcl::PointXYZ &p=p_pro;
         cloud_projected2.at(i)=planeProjection(p,plane2_n,plane2_p);
         pcl::PointXYZI pp=pcl::PointXYZI();
         pp.x=p_pro.x;
         pp.y=p_pro.y;
         pp.z=p_pro.z;
         pp.intensity=0;
     }

     //find third point
     float min_angle2=5.0;
     int min_angle_idx2;
     const pcl::PointXYZ v21 = pcl::PointXYZ(check_pos.x-lotFusPkt.x,check_pos.y-lotFusPkt.y,0);//todo check correct
     std::vector<float> angles2;

       for(unsigned int i=0; i<cloud_projected2.size();++i)
     {
         pcl::PointXYZ &p_pro= cloud_projected2.at(i);
         const pcl::PointXYZ v22 = pcl::PointXYZ(p_pro.x-lotFusPkt.x,p_pro.y-lotFusPkt.y,p_pro.z-lotFusPkt.z);
         float angle= acos( dotProduct(v21,v22)/sqrt(dotProduct(v21,v21)*dotProduct(v22,v22)));
         angles2.push_back(angle);

         if (angle<min_angle2)
         {
             min_angle2=angle;
             min_angle_idx2=i;
           //  ROS_INFO("newminangle : %f ",angle);
         }      
     }


     const pcl::PointXYZ support_point_3= pcl::PointXYZ(cloud_positionRating->at(min_angle_idx2).x,cloud_positionRating->at(min_angle_idx2).y,cloud_positionRating->at(min_angle_idx2).z);

 // */
     viewer.addSphere(support_point_3, 0.02,0,0,1, "sp3", viewport);

     const pcl::PointXYZ final_normal= crossProduct(pcl::PointXYZ(support_point_1.x-support_point_2.x,support_point_1.y-support_point_2.y,support_point_1.z-support_point_2.z),
                                                    pcl::PointXYZ(support_point_1.x-support_point_3.x,support_point_1.y-support_point_3.y,support_point_1.z-support_point_3.z));


     std::vector<unsigned int> convex_hull_indices;
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_positionRating2(new pcl::PointCloud<pcl::PointXYZ>());
     for (unsigned int i = 0; i < cloud_positionRating->size(); i++)
     {
         pcl::PointXYZI& p = cloud_positionRating->at(i);
         const float dist = planeDistance(pcl::PointXYZ(p.x,p.y,p.z),final_normal,support_point_1);
         if(dist<0.01 && dist>-0.01)
         {
             std::string name ="groundContactArea"+boost::lexical_cast<std::string>(i);
             viewer.addSphere(p, 0.01,0,1,1, name, viewport);
             cloud_positionRating2->push_back(pcl::PointXYZ(p.x,p.y,p.z));
         }
         p.intensity=p.z;
     }
     const pcl::PointXYZ pcs=planeProjection(check_pos,final_normal,support_point_1);
     viewer.addSphere(pcs,0.04,1,0,1, "cp_pro", viewport);
     convex_hull_comp(*cloud_positionRating2, convex_hull_indices);
     std::vector<pcl::PointXYZ> convex_hull_points;

     for(int i=0; i<(convex_hull_indices.size()); ++i)
     {
         const pcl::PointXYZ p1(cloud_positionRating2->at(convex_hull_indices[i]).x,cloud_positionRating2->at(convex_hull_indices[i]).y,cloud_positionRating2->at(convex_hull_indices[i]).z);
         convex_hull_points.push_back(p1);
     }


     std::vector<float> rat =computeForceAngleStabilityMetric(check_pos,convex_hull_points);
     for (unsigned int i=0; i<rat.size();++i)
     {

         float c =rat.at(i);
         std::string name ="convex_hull_rating"+boost::lexical_cast<std::string>(convex_hull_indices[i]);
         const pcl::PointXYZ p1(cloud_positionRating2->at(convex_hull_indices[i]).x,cloud_positionRating2->at(convex_hull_indices[i]).y,cloud_positionRating2->at(convex_hull_indices[i]).z);
         const pcl::PointXYZ p2(cloud_positionRating2->at(convex_hull_indices[i+1]).x,cloud_positionRating2->at(convex_hull_indices[i+1]).y,cloud_positionRating2->at(convex_hull_indices[i+1]).z);

         ROS_INFO("RATING r: %f p1: %f %f %f p2:%f %f %f", c,p1.x,p1.y,p1.z,p2.x,p2.y,p2.z);
         viewer.addLine(p1,p2,1.0-c,c,0,name);
     }

    return true;
}

void TerrainClassifier::showPositionRating(pcl::visualization::PCLVisualizer &viewer, const std::string &name, int viewport) const
{


    //draw
    viewer.addSphere(lastRatedPosition, 0.03, "lastRatedPositionSphere", viewport);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud_positionRating, "intensity");
    viewer.addPointCloud<pcl::PointXYZI>(cloud_positionRating,intensity_distribution, "positionRating_cloud", viewport);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name + std::string("_edges"), viewport);


}

}
