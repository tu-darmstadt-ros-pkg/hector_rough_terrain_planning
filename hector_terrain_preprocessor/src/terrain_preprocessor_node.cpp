#include <hector_terrain_preprocessor/terrain_preprocessor_node.h>

#include <pcl/filters/crop_box.h>

namespace hector_terrain_preprocessor
{

bool add(TestModelService::Request &req,
           TestModelService::Response &res)
{
    res.a_srvc_out = req.a_srvc_in + req.testmodel_in.A;
    res.testmodel_out.A = 2*(req.a_srvc_in + req.testmodel_in.A);
    ROS_INFO("request received: x=%ld, y=%ld", (long int)req.a_srvc_in, (long int)req.testmodel_in.A);
    return true;
}

TerrainPreprocessorNode::TerrainPreprocessorNode()
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");

  test_service = nh_.advertiseService("add_two_ints", add);

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
  terrain_preprocessor.reset(new TerrainPreprocessor(params));

  // subscribe topics
  //point_cloud_sub = nh.subscribe("/worldmodel_main/pointcloud_vis", 1, &hector_terrain_preprocessor::setPointCloud, this);
  generate_terrain_model_sub = nh.subscribe("/flor/terrain_preprocessor/generate_terrain_model", 1, &TerrainPreprocessorNode::generateTerrainModel, this);

  // start service clients
  //point_cloud_client = nh.serviceClient<flor_perception_msgs::PointCloudRegionRequest>("/flor/worldmodel/pointcloud_roi");

  // publish topics
  cloud_input_pub = nh.advertise<sensor_msgs::PointCloud2>("/flor/terrain_preprocessor/cloud_input", 1);
  cloud_points_processed_pub = nh.advertise<sensor_msgs::PointCloud2>("/flor/terrain_preprocessor/cloud_processed", 1);
  cloud_points_processed_low_res_pub = nh.advertise<sensor_msgs::PointCloud2>("/flor/terrain_preprocessor/cloud_processed_low_res", 1);
  cloud_points_outfiltered_pub = nh.advertise<sensor_msgs::PointCloud2>("/flor/terrain_preprocessor/cloud_outfiltered", 1);
  cloud_normals_pub = nh.advertise<geometry_msgs::PoseArray>("/flor/terrain_preprocessor/cloud_normals", 1);
  cloud_gradients_pub = nh.advertise<sensor_msgs::PointCloud2>("/flor/terrain_preprocessor/cloud_gradients", 1);
  ground_level_grid_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/flor/terrain_preprocessor/ground_level_grid_map", 1);
  height_grid_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/flor/terrain_preprocessor/height_grid_map", 1);
  mesh_surface_pub = nh.advertise<pcl_msgs::PolygonMesh>("/flor/terrain_preprocessor/mesh_surface", 1);
  terrain_model_pub = nh.advertise<hector_terrain_preprocessor::TerrainModel>("/flor/terrain_preprocessor/terrain_model", 1);

  // start own services
  generate_terrain_model_srv = nh_.advertiseService("/flor/terrain_preprocessor/generate_terrain_model", &TerrainPreprocessorNode::terrainModelService, this);
}

TerrainPreprocessorNode::~TerrainPreprocessorNode()
{
}

void TerrainPreprocessorNode::loadTestPointCloud()
{
//  hector_terrain_preprocessor::TerrainClassifierParams params;
//  terrain_preprocessor->getParams(params);
//  params.ed_radius = 0.05;
//  params.ed_max_std = 0.02;
//  params.ed_non_max_supp_radius = 0.05;
//  params.gg_res = 0.02;
//  terrain_preprocessor->setParams(params);

  ROS_INFO("Load point cloud");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>());
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/hector_terrain_preprocessor/pointclouds/things_on_ground_1_09_25.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/hector_terrain_preprocessor/pointclouds/things_on_ground_2_09_25.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/hector_terrain_preprocessor/pointclouds/things_on_ground_3.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/hector_terrain_preprocessor/pointclouds/things_on_ground_4.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/hector_terrain_preprocessor/pointclouds/things_on_ground_5.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/hector_terrain_preprocessor/pointclouds/traps_ground.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/hector_terrain_preprocessor/pointclouds/ramp.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/hector_terrain_preprocessor/pointclouds/ramp2.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/hector_terrain_preprocessor/pointclouds/rough_terrain_2_4.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/hector_terrain_preprocessor/pointclouds/rough_terrain_2_on_top.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/hector_terrain_preprocessor/pointclouds/zick_zack.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/hector_terrain_preprocessor/pointclouds/new.pcd", *cloud_input);

  pcl::io::loadPCDFile("../pointclouds/ramp2_filtered.pcd", *cloud_input);
  // add filtered point cloud to classifier
  terrain_preprocessor->addCloud(cloud_input);

  generateTerrainModel();
}

bool TerrainPreprocessorNode::terrainModelService(TerrainModelService::Request &req, TerrainModelService::Response &resp)
{
//  if (lower_body_state)
//  {
//    // request new point cloud
//    flor_perception_msgs::PointCloudRegionRequest srv;

//    geometry_msgs::Point min;
//    geometry_msgs::Point max;
//    uint32_t aggregation_size = 0;

//    if (req.terrain_model_request.use_default_region_request)
//    {
//      min = min_bounding_box;
//      max = max_bounding_box;
//      aggregation_size = this->aggregation_size;
//    }
//    else
//    {
//      min = req.terrain_model_request.region_req.bounding_box_min;
//      max = req.terrain_model_request.region_req.bounding_box_max;
//      aggregation_size = req.terrain_model_request.aggregation_size;
//    }

//    srv.request.region_req.header.frame_id = "/world";
//    srv.request.region_req.header.stamp = ros::Time::now();
//    srv.request.region_req.bounding_box_min.x = min.x + lower_body_state->pelvis_pose.position.x;
//    srv.request.region_req.bounding_box_min.y = min.y + lower_body_state->pelvis_pose.position.y;
//    srv.request.region_req.bounding_box_min.z = min.z + std::min(lower_body_state->left_foot_pose.position.z, lower_body_state->right_foot_pose.position.z);
//    srv.request.region_req.bounding_box_max.x = max.x + lower_body_state->pelvis_pose.position.x;
//    srv.request.region_req.bounding_box_max.y = max.y + lower_body_state->pelvis_pose.position.y;
//    srv.request.region_req.bounding_box_max.z = max.z + std::min(lower_body_state->left_foot_pose.position.z, lower_body_state->right_foot_pose.position.z);
//    srv.request.region_req.resolution = 0.0;
//    srv.request.aggregation_size = aggregation_size;

//    if (point_cloud_client.call(srv.request, srv.response))
//      setPointCloud(srv.response.cloud);
//    else
//      ROS_WARN("Point cloud request failed!");
//  }
//  else
    ROS_WARN("Can't request new cloud point due to missing pelvis pose! Using last one if available.");

  // generate terrain model
  if (!generateTerrainModel())
    return false;


  return true;
}

void TerrainPreprocessorNode::setPointCloud(const sensor_msgs::PointCloud2 &cloud_input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(cloud_input, *cloud);

//  pcl::CropBox<pcl::PointXYZ> crop_box_filter;
//  crop_box_filter.setInputCloud(cloud);
//  crop_box_filter.setMin(Eigen::Vector4f(-3.0,-3.0,-1.0,1));
//  crop_box_filter.setMax(Eigen::Vector4f(3.0,3.0,1.0,1));
//  crop_box_filter.filter(*cloud);

  terrain_preprocessor->addCloud(cloud, cloud_input.header.frame_id);

  //ROS_INFO("Saved");
  //pcl::io::savePCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/hector_terrain_preprocessor/pointclouds/new.pcd", cloud_input);
}

void TerrainPreprocessorNode::generateTerrainModel(const TerrainModelRequest &req)
{
    assert(0); //temp hack
 // TerrainModelService srv;
 // srv.request.terrain_model_request = req;
 // terrainModelService(srv.request, srv.response);
}

bool TerrainPreprocessorNode::generateTerrainModel()
{


  publishResult();
  return true;
}

void TerrainPreprocessorNode::publishResult() const
{
  sensor_msgs::PointCloud2 cloud_point_msg;

  if (cloud_input_pub.getNumSubscribers() > 0)
  {
    pcl::toROSMsg(*(terrain_preprocessor->getCloudInput()), cloud_point_msg);
    cloud_point_msg.header.stamp = ros::Time::now();
    cloud_point_msg.header.frame_id = terrain_preprocessor->getFrameId();
    cloud_input_pub.publish(cloud_point_msg);
  }



  if (cloud_points_processed_low_res_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    terrain_preprocessor->getCloudProcessedLowRes(cloud);
    pcl::toROSMsg(*cloud, cloud_point_msg);
    cloud_point_msg.header.stamp = ros::Time::now();
    cloud_point_msg.header.frame_id = terrain_preprocessor->getFrameId();
    cloud_points_processed_low_res_pub.publish(cloud_point_msg);
  }

  if (cloud_points_outfiltered_pub.getNumSubscribers() > 0)
  {
    pcl::toROSMsg(*(terrain_preprocessor->getCloudOutfiltered()), cloud_point_msg);
    cloud_point_msg.header.stamp = ros::Time::now();
    cloud_point_msg.header.frame_id = terrain_preprocessor->getFrameId();
    cloud_points_outfiltered_pub.publish(cloud_point_msg);
  }



}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hector_terrain_preprocessor_node");

  hector_terrain_preprocessor::TerrainPreprocessorNode terrain_preprocessor_node;

  for (int i = 1; i < argc; ++i)
  {
    if (std::string(argv[i]) == "-loadTestCloud")
      terrain_preprocessor_node.loadTestPointCloud();
  }

  ros::spin();

  return 0;
}