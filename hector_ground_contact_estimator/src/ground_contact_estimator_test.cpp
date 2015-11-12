#include <hector_ground_contact_estimator/ground_contact_estimator.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace hector_ground_contact_estimator;
class TerrainClassifierTest
{
public:
    TerrainClassifierTest()
    {}

    void initialPoseCb(geometry_msgs::PoseWithCovarianceStamped msg);
    void test_terrain_classifier();
    void test_terrain_classifier_standalone();
    bool new_position;
    pcl::PointXYZ check_pos;
    float orientation;

};

void TerrainClassifierTest::initialPoseCb(geometry_msgs::PoseWithCovarianceStamped msg)
{
    check_pos.x = msg.pose.pose.position.x;
    check_pos.y = msg.pose.pose.position.y;
    orientation = 2. * acos(msg.pose.pose.orientation.z);
    ROS_INFO("reveiced pose x %f y %f orient %f",check_pos.x,check_pos.y,orientation);
    new_position = true;

}

void TerrainClassifierTest::test_terrain_classifier()
{
    ros::NodeHandle nh;
    new_position = false;
    ros::Publisher cloud_input_pub = nh.advertise<sensor_msgs::PointCloud2>("/flor/terrain_classifier/cloud_input", 3);
    ros::Subscriber initialpose_sub = nh.subscribe("/initialpose", 100, &TerrainClassifierTest::initialPoseCb, this);

    ROS_INFO("Load point cloud");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::io::loadPCDFile("../pointclouds/left_obstacle_start_zero.pcd", *cloud_original);
    //pcl::io::loadPCDFile("../pointclouds/konststeigend_x.pcd", *cloud_original);
    //pcl::io::loadPCDFile("../pointclouds/zwei_ebenen_steigend.pcd", *cloud_original);
    //pcl::io::loadPCDFile("../pointclouds/dach.pcd", *cloud_original);
    pcl::io::loadPCDFile("../pointclouds/pc_barrier_left_close.pcd", *cloud_original);
    //pcl::io::loadPCDFile("../pointclouds/stairs_ramp_map.pcd", *cloud_original);
    //pcl::io::loadPCDFile("../pointclouds/big_sim.pcd", *cloud_original);

    hector_ground_contact_estimator::GroundContactEstimator terrain_model(*cloud_original);

    // visualization
    pcl::visualization::PCLVisualizer viewer("Terrain classifier");
    viewer.initCameraParameters();

    int view_port_1(0);
    viewer.createViewPort(0.0, 0.5, 0.5, 1.0, view_port_1);
    viewer.addCoordinateSystem(0.5, view_port_1);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_original, "input cloud", view_port_1);

    int view_port_2(0);
    viewer.createViewPort(0.5, 0.5, 1.0, 1.0, view_port_2);
    viewer.addCoordinateSystem(0.5, view_port_2);

    int view_port_3(0);
    viewer.createViewPort(0.0, 0.0, 0.5, 0.5, view_port_3);
    viewer.addCoordinateSystem(0.5, view_port_3);

    int view_port_4(0);
    viewer.createViewPort(0.5, 0.0, 1.0, 0.5, view_port_4);
    viewer.addCoordinateSystem(0.5, view_port_4);

    ROS_INFO("Compute Position rating... ports %i %i %i %i",view_port_1,view_port_2,view_port_3,view_port_4);


    // Position, Orientation (in radiants)
    float x, y;
    //DEBUG for Simulation (file: big_sim.pcd)
    //x  = 1.511102; y = 1.011102; orientation = 0.7854; // bpr is 2.80 this is a normal position
    //x = 1.400002; y = 0.900002; orientation = 0.7854; // convex_hull_quickfix
    //x = 2.450002; y = 2.038902; orientation = 1.5708; // after 15 iterations position to check not in supporting polygon TOO STEEP
    //x = 2.4; y = 1.55; orientation = 1.570796; // too low pos rating
    //x = 3.600002; y = 3.000002; orientation = 1.570796; // posrating = 0.0 should not be!
    //testpos     [ INFO] [1445513760.468928464]: reveiced pose x 4.420969 y 0.157174 orient 2.458588
    x = 4.954349; y = 1.751206; orientation = 1.658275;


    check_pos = pcl::PointXYZ(x, y, 0.0);
    //float orientation = (0.0)/180.0*3.14;
    float position_rating = 10.0;
    int unstable_axis = 10;
    pcl::PointXYZ pc, p0, p1, p2, p3;

    pcl::visualization::PCLVisualizer* viewer_ptr = new pcl::visualization::PCLVisualizer(viewer);
    terrain_model.computePositionRating(check_pos, orientation, pc, p0, p1, p2, p3,
                                        position_rating, unstable_axis, viewer_ptr, view_port_1, view_port_2, view_port_3, view_port_4, true);


    sensor_msgs::PointCloud2 cloud_point_msg;
    ROS_INFO("TerrainClassifierTest publishResult");
    if (cloud_input_pub.getNumSubscribers() >= 0)
    {
        ROS_INFO("TerrainClassifierNode publish cloud input");
        pcl::toROSMsg(*(cloud_original), cloud_point_msg);
        cloud_point_msg.header.stamp = ros::Time::now();
        cloud_point_msg.header.frame_id = "world";//terrain_classifier->getFrameId();
        cloud_input_pub.publish(cloud_point_msg);
    }
    bool subscribed = false;


    while (!viewer.wasStopped())
    {

        if (cloud_input_pub.getNumSubscribers() == 0)
        {
            cloud_input_pub.publish(cloud_point_msg);
        }
        else if(subscribed == false)
        {
            cloud_input_pub.publish(cloud_point_msg);
            subscribed = true;
        }
        if(new_position)
        {
            new_position = false;
            viewer.removeAllShapes();
            terrain_model.computePositionRating(check_pos, orientation, pc, p0, p1, p2, p3,
                                                position_rating, unstable_axis, viewer_ptr, view_port_1, view_port_2, view_port_3, view_port_4, true);

        }
        viewer.spinOnce(100);
        ros::spinOnce();
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

void TerrainClassifierTest::test_terrain_classifier_standalone()
{
    ros::NodeHandle nh;
    new_position = false;
    ros::Publisher cloud_input_pub = nh.advertise<sensor_msgs::PointCloud2>("/flor/terrain_classifier/cloud_input", 3);
    ros::Subscriber initialpose_sub = nh.subscribe("/initialpose", 100, &TerrainClassifierTest::initialPoseCb, this);

    ROS_INFO("Load point cloud");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::io::loadPCDFile("../pointclouds/left_obstacle_start_zero.pcd", *cloud_original);
    //pcl::io::loadPCDFile("../pointclouds/konststeigend_x.pcd", *cloud_original);
    //pcl::io::loadPCDFile("../pointclouds/zwei_ebenen_steigend.pcd", *cloud_original);
    //pcl::io::loadPCDFile("../pointclouds/dach.pcd", *cloud_original);
    //pcl::io::loadPCDFile("../pointclouds/ramp2_filtered.pcd", *cloud_original);
    pcl::io::loadPCDFile("../pointclouds/stairs_ramp_map.pcd", *cloud_original);
    //pcl::io::loadPCDFile("../pointclouds/big_sim.pcd", *cloud_original);

    hector_ground_contact_estimator::GroundContactEstimator terrain_model(*cloud_original);

    // Position, Orientation (in radiants)
    float x, y;
    x = 4.954349; y = 1.751206; orientation = 1.658275;
    check_pos = pcl::PointXYZ(x, y, 0.0);
    //float orientation = (0.0)/180.0*3.14;
    float position_rating = 10.0;
    int unstable_axis = 10;
    pcl::PointXYZ pc, p0, p1, p2, p3;

    terrain_model.computePositionRating(check_pos, orientation, pc, p0, p1, p2, p3,
                                        position_rating, unstable_axis);
    ROS_INFO("Rating %f",position_rating);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "terrain_test");
    ros::Time::init();

   TerrainClassifierTest test;
    test.test_terrain_classifier();

    return 0;
}

