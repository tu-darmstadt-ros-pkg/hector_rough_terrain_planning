#include <flor_terrain_classifier/terrain_classifier_node.h>

#include <pcl/filters/crop_box.h>

namespace flor_terrain_classifier
{

bool add(TestModelService::Request &req,
           TestModelService::Response &res)
{
    res.a_srvc_out = req.a_srvc_in + req.testmodel_in.A;
    res.testmodel_out.A = 2*(req.a_srvc_in + req.testmodel_in.A);
    ROS_INFO("request received: x=%ld, y=%ld", (long int)req.a_srvc_in, (long int)req.testmodel_in.A);
    return true;
}

TerrainClassifierNode::TerrainClassifierNode()
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
  terrain_classifier.reset(new TerrainClassifier(params));

  // subscribe topics
  //point_cloud_sub = nh.subscribe("/worldmodel_main/pointcloud_vis", 1, &TerrainClassifierNode::setPointCloud, this);
  generate_terrain_model_sub = nh.subscribe("/flor/terrain_classifier/generate_terrain_model", 1, &TerrainClassifierNode::generateTerrainModel, this);

  // start service clients
  //point_cloud_client = nh.serviceClient<flor_perception_msgs::PointCloudRegionRequest>("/flor/worldmodel/pointcloud_roi");

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
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/flor_terrain_classifier/pointclouds/rough_terrain_2_4.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/flor_terrain_classifier/pointclouds/rough_terrain_2_on_top.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/flor_terrain_classifier/pointclouds/zick_zack.pcd", *cloud_input);
  //pcl::io::loadPCDFile("/home/alex/flor/rosbuild_ws/vigir_perception/vigir_worldmodel/flor_terrain_classifier/pointclouds/new.pcd", *cloud_input);

  //pcl::io::loadPCDFile("/opt/hector/hydro/stacks/hector_rough_terrain_planning/flor_terrain_classifier/pointclouds/ramp2_filtered.pcd", *cloud_input);

  pcl::io::loadPCDFile("/opt/hector/hydro/stacks/hector_rough_terrain_planning/flor_terrain_classifier/pointclouds/left_obstacle_start_zero.pcd", *cloud_input);

  //add filtered point cloud to classifier
  terrain_classifier->addCloud(cloud_input);

  generateTerrainModel();
}

bool TerrainClassifierNode::terrainModelService(TerrainModelService::Request &req, TerrainModelService::Response &resp)
{
  // generate terrain model
  if (!generateTerrainModel())    
  {
      ROS_WARN("TerrainClassifierNode::terrainModelService failed");
      return false;
  }
  return true;
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


  publishResult();
  return true;
}

void TerrainClassifierNode::publishResult() const
{
  sensor_msgs::PointCloud2 cloud_point_msg;

  ROS_INFO("TerrainClassifierNode publishResult");
  if (cloud_input_pub.getNumSubscribers() > 0)
  {
    ROS_INFO("TerrainClassifierNode publish cloud input");
    pcl::toROSMsg(*(terrain_classifier->getCloudInput()), cloud_point_msg);
    cloud_point_msg.header.stamp = ros::Time::now();
    cloud_point_msg.header.frame_id = terrain_classifier->getFrameId();
    cloud_input_pub.publish(cloud_point_msg);
  }



  if (cloud_points_processed_low_res_pub.getNumSubscribers() > 0)
  {
      ROS_INFO("TerrainClassifierNode publish cloud lowres");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    terrain_classifier->getCloudProcessedLowRes(cloud);
    pcl::toROSMsg(*cloud, cloud_point_msg);
    cloud_point_msg.header.stamp = ros::Time::now();
    cloud_point_msg.header.frame_id = terrain_classifier->getFrameId();
    cloud_points_processed_low_res_pub.publish(cloud_point_msg);
  }

  if (cloud_points_outfiltered_pub.getNumSubscribers() > 0)
  {
      ROS_INFO("TerrainClassifierNode publish cloud outfiltered");
    pcl::toROSMsg(*(terrain_classifier->getCloudOutfiltered()), cloud_point_msg);
    cloud_point_msg.header.stamp = ros::Time::now();
    cloud_point_msg.header.frame_id = terrain_classifier->getFrameId();
    cloud_points_outfiltered_pub.publish(cloud_point_msg);
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
