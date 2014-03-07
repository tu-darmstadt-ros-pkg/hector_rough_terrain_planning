#include <flor_terrain_classifier/terrain_classifier_node.h>

#include <pcl/filters/crop_box.h>

namespace flor_terrain_classifier
{
TerrainClassifierNode::TerrainClassifierNode()
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");

  // load parameters
  nh_.param("point_cloud/min_bounding_box/x", min_bounding_box.x, -1.0);
  nh_.param("point_cloud/min_bounding_box/y", min_bounding_box.y, -1.0);
  nh_.param("point_cloud/min_bounding_box/z", min_bounding_box.z, -1.0);
  nh_.param("point_cloud/max_bounding_box/x", max_bounding_box.x, 1.0);
  nh_.param("point_cloud/max_bounding_box/y", max_bounding_box.y, 1.0);
  nh_.param("point_cloud/max_bounding_box/z", max_bounding_box.z, 1.0);
  nh_.param("point_cloud/aggregation_size", (int&)aggregation_size, 2000);

  // init terrain classifier
  TerrainClassifierParams params(nh_);
  terrain_classifier.reset(new TerrainClassifier(params));

  // subscribe topics
  //point_cloud_sub = nh.subscribe("/worldmodel_main/pointcloud_vis", 1, &TerrainClassifierNode::setPointCloud, this);
  lower_body_state_sub = nh.subscribe("/flor/state/lower_body_world", 1, &TerrainClassifierNode::setLowerBodyState, this);
  generate_terrain_model_sub = nh.subscribe("/flor/terrain_classifier/generate_terrain_model", 1, &TerrainClassifierNode::generateTerrainModel, this);

  // start service clients
  point_cloud_client = nh.serviceClient<flor_perception_msgs::PointCloudRegionRequest>("/flor/worldmodel/pointcloud_roi");

  // publish topics
  cloud_input_pub = nh.advertise<sensor_msgs::PointCloud2>("/flor/terrain_classifier/cloud_input", 1);
  cloud_points_processed_pub = nh.advertise<sensor_msgs::PointCloud2>("/flor/terrain_classifier/cloud_processed", 1);
  cloud_points_processed_low_res_pub = nh.advertise<sensor_msgs::PointCloud2>("/flor/terrain_classifier/cloud_processed_low_res", 1);
  cloud_points_outfiltered_pub = nh.advertise<sensor_msgs::PointCloud2>("/flor/terrain_classifier/cloud_outfiltered", 1);
  cloud_normals_pub = nh.advertise<geometry_msgs::PoseArray>("/flor/terrain_classifier/cloud_normals", 1);
  cloud_gradients_pub = nh.advertise<sensor_msgs::PointCloud2>("/flor/terrain_classifier/cloud_gradients", 1);
  ground_level_grid_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/flor/terrain_classifier/ground_level_grid_map", 1);
  height_grid_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/flor/terrain_classifier/height_grid_map", 1);
  mesh_surface_pub = nh.advertise<pcl_msgs::PolygonMesh>("/flor/terrain_classifier/mesh_surface", 1);
  terrain_model_pub = nh.advertise<flor_terrain_classifier::TerrainModel>("/flor/terrain_classifier/terrain_model", 1);

  // start own services
  generate_terrain_model_srv = nh_.advertiseService("/flor/terrain_classifier/generate_terrain_model", &TerrainClassifierNode::terrainModelService, this);
}

TerrainClassifierNode::~TerrainClassifierNode()
{
}

void TerrainClassifierNode::loadTestPointCloud()
{
//  flor_terrain_classifier::TerrainClassifierParams params;
//  terrain_classifier->getParams(params);
//  params.ed_radius = 0.05;
//  params.ed_max_std = 0.02;
//  params.ed_non_max_supp_radius = 0.05;
//  params.gg_res = 0.02;
//  terrain_classifier->setParams(params);

  ROS_INFO("Load point cloud");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>());
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/flor_terrain_classifier/pointclouds/things_on_ground_1_09_25.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/flor_terrain_classifier/pointclouds/things_on_ground_2_09_25.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/flor_terrain_classifier/pointclouds/things_on_ground_3.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/flor_terrain_classifier/pointclouds/things_on_ground_4.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/flor_terrain_classifier/pointclouds/things_on_ground_5.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/flor_terrain_classifier/pointclouds/traps_ground.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/flor_terrain_classifier/pointclouds/ramp.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/flor_terrain_classifier/pointclouds/ramp2.pcd", *cloud_input);
  pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/flor_terrain_classifier/pointclouds/rough_terrain_2_4.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/flor_terrain_classifier/pointclouds/rough_terrain_2_on_top.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/flor_terrain_classifier/pointclouds/zick_zack.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/flor_terrain_classifier/pointclouds/new.pcd", *cloud_input);

  // add filtered point cloud to classifier
  terrain_classifier->addCloud(cloud_input);

  generateTerrainModel();
}

bool TerrainClassifierNode::terrainModelService(TerrainModelService::Request &req, TerrainModelService::Response &resp)
{
//  // generate terrain model
//  if (!generateTerrainModel())
//    return false;

//  pcl::toROSMsg(*(terrain_classifier->getPointsWithsNormals()), resp.terrain_model.cloud_points_with_normals);
//  resp.terrain_model.ground_level_map = *(terrain_classifier->getGroundLevelGridMap());
//  resp.terrain_model.height_map = *(terrain_classifier->getHeightGridMap(resp.terrain_model.height_map_scale));

//  return true;

  if (lower_body_state)
  {
    // request new point cloud
    flor_perception_msgs::PointCloudRegionRequest srv;

    geometry_msgs::Point min;
    geometry_msgs::Point max;
    uint32_t aggregation_size = 0;

    if (req.terrain_model_request.use_default_region_request)
    {
      min = min_bounding_box;
      max = max_bounding_box;
      aggregation_size = this->aggregation_size;
    }
    else
    {
      min = req.terrain_model_request.region_req.bounding_box_min;
      max = req.terrain_model_request.region_req.bounding_box_max;
      aggregation_size = req.terrain_model_request.aggregation_size;
    }

    srv.request.region_req.header.frame_id = "/world";
    srv.request.region_req.header.stamp = ros::Time::now();
    srv.request.region_req.bounding_box_min.x = min.x + lower_body_state->pelvis_pose.position.x;
    srv.request.region_req.bounding_box_min.y = min.y + lower_body_state->pelvis_pose.position.y;
    srv.request.region_req.bounding_box_min.z = min.z + std::min(lower_body_state->left_foot_pose.position.z, lower_body_state->right_foot_pose.position.z);
    srv.request.region_req.bounding_box_max.x = max.x + lower_body_state->pelvis_pose.position.x;
    srv.request.region_req.bounding_box_max.y = max.y + lower_body_state->pelvis_pose.position.y;
    srv.request.region_req.bounding_box_max.z = max.z + std::min(lower_body_state->left_foot_pose.position.z, lower_body_state->right_foot_pose.position.z);
    srv.request.region_req.resolution = 0.0;
    srv.request.aggregation_size = aggregation_size;

    if (point_cloud_client.call(srv.request, srv.response))
      setPointCloud(srv.response.cloud);
    else
      ROS_WARN("Point cloud request failed!");
  }
  else
    ROS_WARN("Can't request new cloud point due to missing pelvis pose! Using last one if available.");

  // generate terrain model
  if (!generateTerrainModel())
    return false;

  pcl::toROSMsg(*(terrain_classifier->getPointsWithsNormals()), resp.terrain_model.cloud_points_with_normals);
  resp.terrain_model.ground_level_map = *(terrain_classifier->getGroundLevelGridMap());
  resp.terrain_model.height_map = *(terrain_classifier->getHeightGridMap(resp.terrain_model.height_map_scale));

  return true;
}

void TerrainClassifierNode::setLowerBodyState(const flor_state_msgs::LowerBodyStateConstPtr &lower_body_state)
{
  this->lower_body_state = lower_body_state;
  this->terrain_classifier->setLowerBodyState(lower_body_state);
}

void TerrainClassifierNode::setPointCloud(const sensor_msgs::PointCloud2 &cloud_input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(cloud_input, *cloud);

//  pcl::CropBox<pcl::PointXYZ> crop_box_filter;
//  crop_box_filter.setInputCloud(cloud);
//  crop_box_filter.setMin(Eigen::Vector4f(-3.0,-3.0,-1.0,1));
//  crop_box_filter.setMax(Eigen::Vector4f(3.0,3.0,1.0,1));
//  crop_box_filter.filter(*cloud);

  terrain_classifier->addCloud(cloud, cloud_input.header.frame_id);

  //ROS_INFO("Saved");
  //pcl::io::savePCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/flor_terrain_classifier/pointclouds/new.pcd", cloud_input);
}

void TerrainClassifierNode::generateTerrainModel(const TerrainModelRequest &req)
{
  TerrainModelService srv;
  srv.request.terrain_model_request = req;
  terrainModelService(srv.request, srv.response);
}

bool TerrainClassifierNode::generateTerrainModel()
{
  // generate gradients
  ROS_INFO("Generate normals and gradients...");
  if (!terrain_classifier->computeGradients())
    return false;

  // detect edges
  ROS_INFO("Detect edges...");
  if (!terrain_classifier->detectEdges())
    return false;

  // generate height grid map
  ROS_INFO("Generate height grid map...");
  if (!terrain_classifier->generateHeightGridmap())
    return false;

  // generate ground level grid map
  ROS_INFO("Generate ground level grid map...");
  if (!terrain_classifier->generateGroundLevelGridmap())
    return false;

  publishResult();
  return true;
}

void TerrainClassifierNode::publishResult() const
{
  sensor_msgs::PointCloud2 cloud_point_msg;

  if (cloud_input_pub.getNumSubscribers() > 0)
  {
    pcl::toROSMsg(*(terrain_classifier->getCloudInput()), cloud_point_msg);
    cloud_point_msg.header.stamp = ros::Time::now();
    cloud_point_msg.header.frame_id = terrain_classifier->getFrameId();
    cloud_input_pub.publish(cloud_point_msg);
  }

  if (cloud_points_processed_pub.getNumSubscribers() > 0)
  {
    pcl::toROSMsg(*(terrain_classifier->getPointsWithsNormals()), cloud_point_msg);
    cloud_point_msg.header.stamp = ros::Time::now();
    cloud_point_msg.header.frame_id = terrain_classifier->getFrameId();
    cloud_points_processed_pub.publish(cloud_point_msg);
  }

  if (cloud_points_processed_low_res_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    terrain_classifier->getCloudProcessedLowRes(cloud);
    pcl::toROSMsg(*cloud, cloud_point_msg);
    cloud_point_msg.header.stamp = ros::Time::now();
    cloud_point_msg.header.frame_id = terrain_classifier->getFrameId();
    cloud_points_processed_low_res_pub.publish(cloud_point_msg);
  }

  if (cloud_points_outfiltered_pub.getNumSubscribers() > 0)
  {
    pcl::toROSMsg(*(terrain_classifier->getCloudOutfiltered()), cloud_point_msg);
    cloud_point_msg.header.stamp = ros::Time::now();
    cloud_point_msg.header.frame_id = terrain_classifier->getFrameId();
    cloud_points_outfiltered_pub.publish(cloud_point_msg);
  }

  if (cloud_normals_pub.getNumSubscribers() > 0)
  {
    // convert normals to PoseArray and publish them
    const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_points_with_normals(terrain_classifier->getPointsWithsNormals());

    geometry_msgs::PoseArray pose_array;
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = terrain_classifier->getFrameId();

    geometry_msgs::Pose pose_msg;
    for (size_t i = 0; i < cloud_points_with_normals->size(); i++)
    {
      const pcl::PointNormal &p_n = cloud_points_with_normals->at(i);

      pose_msg.position.x = p_n.x;
      pose_msg.position.y = p_n.y;
      pose_msg.position.z = p_n.z;

      double r = asin(p_n.normal_x);
      double p = -asin(p_n.normal_y);
      double y = 0.0;//asin(p_n.normal_z);

      tf::quaternionTFToMsg(tf::createQuaternionFromRPY(r, p, y), pose_msg.orientation);

      pose_array.poses.push_back(pose_msg);
    }

    cloud_normals_pub.publish(pose_array);
  }

  if (cloud_gradients_pub.getNumSubscribers() > 0)
  {
    pcl::toROSMsg(*(terrain_classifier->getGradients()), cloud_point_msg);
    cloud_point_msg.header.stamp = ros::Time::now();
    cloud_point_msg.header.frame_id = terrain_classifier->getFrameId();
    cloud_gradients_pub.publish(cloud_point_msg);
  }

  // publish ground level grid map
  if (ground_level_grid_map_pub.getNumSubscribers() > 0)
    ground_level_grid_map_pub.publish(terrain_classifier->getGroundLevelGridMap());

  // publish height grid map
  if (height_grid_map_pub.getNumSubscribers() > 0)
    height_grid_map_pub.publish(terrain_classifier->getHeightGridMap());

  // publish mesh surface
  if (mesh_surface_pub.getNumSubscribers() > 0)
  {
    pcl_msgs::PolygonMesh mesh_msg;
    pcl_conversions::fromPCL(*(terrain_classifier->getMeshSurface()), mesh_msg);
    mesh_msg.header.stamp = ros::Time::now();
    mesh_msg.header.frame_id = terrain_classifier->getFrameId();
    mesh_surface_pub.publish(mesh_msg);
  }

  // publish terrain model
  if (terrain_model_pub.getNumSubscribers() > 0)
  {
    flor_terrain_classifier::TerrainModel model;
    pcl::toROSMsg(*(terrain_classifier->getPointsWithsNormals()), model.cloud_points_with_normals);
    model.ground_level_map = *(terrain_classifier->getGroundLevelGridMap());
    model.height_map = *(terrain_classifier->getHeightGridMap(model.height_map_scale));
    terrain_model_pub.publish(model);
  }
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flor_terrain_classifier_node");

  flor_terrain_classifier::TerrainClassifierNode terrain_classifier_node;

  for (int i = 1; i < argc; ++i)
  {
    if (std::string(argv[i]) == "-loadTestCloud")
      terrain_classifier_node.loadTestPointCloud();
  }

  ros::spin();

  return 0;
}
