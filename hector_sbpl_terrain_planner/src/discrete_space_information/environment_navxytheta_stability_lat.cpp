//Implementation for new environment, analogous to https://github.com/sbpl/sbpl/tree/master/src/discrete_space_information

#include <hector_sbpl_terrain_planner/discrete_space_information/environment_navxytheta_stability_lat.h>
#include <cstdio>
#include <ctime>
#include <sbpl/utils/key.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_msgs/TFMessage.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cstdlib>

#include <flor_terrain_classifier/TestModelService.h>
#include <flor_terrain_classifier/TestModel.h>
#include <flor_terrain_classifier/TerrainModel.h>
#include <flor_terrain_classifier/TerrainModelService.h>
#include <flor_terrain_classifier/terrain_classifier.h>
#define _USE_MATH_DEFINES
#include <math.h>
using namespace std;

#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
#endif

static long int checks = 0;
static int temp = 0;

//-----------------constructors/destructors-------------------------------

void EnvironmentNAVXYTHETASTAB::terrainModelCallback(const sensor_msgs::PointCloud2 msg)
{
    if(!receivedWorldmodelPC)
    {
        receivedWorldmodelPC=true;
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(msg, pcl_pc);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromPCLPointCloud2(pcl_pc, cloud);
        terrainModel = hector_terrain_model::TerrainModel(cloud);
        ROS_INFO("cloudPTR in terrainModel size %lu", terrainModel.cloud_processed.size());
        flat_position_rating = terrainModel.bestPosRating()*0.70;

        ROS_INFO("min_position_rating = %f", flat_position_rating);
        sleep(1);
        sensor_msgs::PointCloud2 cloud_point_msg;
        pcl::toROSMsg(cloud, cloud_point_msg);
        cloud_point_msg.header.stamp = ros::Time::now();
        cloud_point_msg.header.frame_id = "map";
        pubTerrainModel.publish(cloud_point_msg);
    }
    else{
        ROS_INFO("entered Callback, world model was received before");
        //ROS_INFO("cloud_processed_Ptr in terrainModel: size = %i", terrainModel.cloud_processed_Ptr->size());
    }
}

void EnvironmentNAVXYTHETASTAB::mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
{
    t_lastMapPos_ = map->header.stamp;
    map_center_map=map->info.origin.position;
}


void EnvironmentNAVXYTHETASTAB::tfCallback(const tf2_msgs::TFMessage& msg){
}

// for path checking receive path and pcl
void EnvironmentNAVXYTHETASTAB::octomap_point_cloud_centers_Callback(const sensor_msgs::PointCloud2 msg){

    bool writepcl = false;
    temp = temp + 1;
    //ROS_INFO("temp = %i", temp);

    if(!receivedWorldmodelPC)
    {
        receivedWorldmodelPC=true;
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(msg, pcl_pc);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromPCLPointCloud2(pcl_pc, cloud);
        terrainModel = hector_terrain_model::TerrainModel(cloud);
        ROS_INFO("cloudPTR in terrainModel size %lu", terrainModel.cloud_processed.size());
        flat_position_rating = terrainModel.bestPosRating()*0.95;

        //ROS_INFO("min_position_rating = %f", flat_position_rating);
        //sleep(1);
        //sensor_msgs::PointCloud2 cloud_point_msg;
        //pcl::toROSMsg(cloud, cloud_point_msg);
        //cloud_point_msg.header.stamp = ros::Time::now();
        //cloud_point_msg.header.frame_id = "map";
        //terrainModelPublisher.publish(cloud_point_msg);
    }
    else{
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(msg, pcl_pc);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromPCLPointCloud2(pcl_pc, cloud);
        if (temp == 40 && writepcl){
            ROS_INFO("write plc");
            pcl::io::savePCDFile("test_pcl.pcd", cloud, false);
            ROS_INFO("writing done");
        }
        terrainModel.updateCloud(cloud);
    }
}

void rotatePoint(pcl::PointXYZ& p, float degree /*radiants*/){
    float px = cos(degree)*p.x - sin(degree)*p.y;
    float py = sin(degree)*p.x + cos(degree)*p.y;
    float pz = p.z;
    p.x = px; p.y = py; p.z= pz;
}

void EnvironmentNAVXYTHETASTAB::pathCallback(const nav_msgs::Path msg){

    ROS_INFO("New path with %lu poses, bpr = %f", msg.poses.size(), terrainModel.bestPosRating());
    if (receivedWorldmodelPC){

        int display_every_x_poses = 10; // display every x poses from the path.
        visualization_msgs::MarkerArray marker_array;
        float l = terrainModel.robot_length /2.0; // length of rectangle (x) / 2
        float w = terrainModel.robot_width / 2.0; // width of rectangle (y) / 2
        float tfz = 0.3; // transformation from world to fixframe
        float ivr = terrainModel.invalid_rating; // between 0 and 0.4
        float br = terrainModel.bestPosRating();

        visualization_msgs::Marker marker_linelist;
        marker_linelist.header.frame_id = "world";
        marker_linelist.header.stamp = ros::Time();
        marker_linelist.ns = "rectangles";
        marker_linelist.id = 0;
        marker_linelist.type = visualization_msgs::Marker::LINE_LIST;
        marker_linelist.action = visualization_msgs::Marker::ADD;

        for(unsigned int i = 0; i < msg.poses.size(); i = i + display_every_x_poses){
            geometry_msgs::PoseStamped poseStamped = msg.poses[i];
            geometry_msgs::Pose pose = poseStamped.pose;
            float px = pose.position.x;
            float py = pose.position.y;
            float pz = pose.position.z;
            if (i == 0){ // publish robot marker on its spot
                visualization_msgs::Marker robot_pose_msg;
                robot_pose_msg.header.frame_id = "world";
                robot_pose_msg.header.stamp = ros::Time();
                robot_pose_msg.id = 0;
                robot_pose_msg. type = visualization_msgs::Marker::SPHERE;
                robot_pose_msg.action = visualization_msgs::Marker::ADD;
                robot_pose_msg.pose.position.x = px; //1; // px
                robot_pose_msg.pose.position.y = py; // 1; // py
                robot_pose_msg.pose.position.z = pz + 0.45;
                robot_pose_msg.pose.orientation.x = 0.0; //TODO
                robot_pose_msg.pose.orientation.y = 0.0;
                robot_pose_msg.pose.orientation.z = 0.0;
                robot_pose_msg.pose.orientation.w = 1.0;
                robot_pose_msg.scale.x = 0.2;
                robot_pose_msg.scale.y = 0.2;
                robot_pose_msg.scale.z = 0.2;
                robot_pose_msg.color.a = 1;
                robot_pose_msg.color.r = 1.0;
                robot_pose_msg.color.g = 1.0;
                robot_pose_msg.color.b = 1.0;

                pubRobotMarker.publish(robot_pose_msg);

            }
            pcl::PointXYZ checkpos = pcl::PointXYZ(px, py, pz);

            // conversion from Quaternion to yaw (en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)
            geometry_msgs::Quaternion q = pose.orientation;
            float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
           // float roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(pow(q1,2)+pow(q2,2)));
           // float pitch = asin(2*(q0*q2 - q3*q1));

            float yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(pow(q2,2)+pow(q3,2))); // TODO check
            float orientation = yaw;

            float position_rating;
            int instable_axis_unused;
            pcl::PointXYZ pc, p0, p1, p2, p3;
            if (false == terrainModel.computePositionRating(checkpos, orientation, pc, p0, p1, p2, p3, position_rating, instable_axis_unused)){
                position_rating = 0.0;
                ROS_INFO("computePositionRating returned FALSE for the next shown position");
                instable_axis_unused = 0;
                p0.x = l; p0.y = w; p0.z = tfz;
                p1.x = -l; p1.y = w; p1.z = tfz;
                p2.x = -l; p2.y = -w; p2.z = tfz;
                p3.x = l; p3.y = -w; p3.z = tfz;
                rotatePoint(p0, orientation);
                rotatePoint(p1, orientation);
                rotatePoint(p2, orientation);
                rotatePoint(p3, orientation);
                p0.x = p0.x + px; p0.y = p0.y + py; p0.z = p0.z + pz;
                p1.x = p1.x + px; p1.y = p1.y + py; p1.z = p1.z + pz;
                p2.x = p2.x + px; p2.y = p2.y + py; p2.z = p2.z + pz;
                p3.x = p3.x + px; p3.y = p3.y + py; p3.z = p3.z + pz;
            }
            ROS_INFO("POSITION RATING = %f, x = %f, y = %f, angle = %f", position_rating, px, py, orientation);


            marker_linelist.pose.position.x = 0; //1; // px
            marker_linelist.pose.position.y = 0; // 1; // py
            marker_linelist.pose.position.z = 0; // 1; // pz
            marker_linelist.pose.orientation.x = 0.0;
            marker_linelist.pose.orientation.y = 0.0;
            marker_linelist.pose.orientation.z = 0.0; // orientation? //TODO
            marker_linelist.pose.orientation.w = 1.0;
            marker_linelist.scale.x = 0.03; //width
            marker_linelist.scale.y = 0.25; //length
            marker_linelist.scale.z = 0.01;
            marker_linelist.color.a = 1.0;
            marker_linelist.color.r = 1.0;
            marker_linelist.color.g = 1.0;
            marker_linelist.color.b = 1.0;
            std_msgs::ColorRGBA rating_color;


            if(position_rating<ivr){
                rating_color.a = 1.0;
                rating_color.r = 1.0;
                rating_color.g = 0.0;
                rating_color.b = position_rating/ivr;
            }
            else if (position_rating > br/2){
                rating_color.a = 1.0;
                rating_color.r = 0.0;
                rating_color.g = 1.0;
                rating_color.b = 0.0;
            }
            else{
                rating_color.a = 1.0;
                rating_color.r = 0.0;
                rating_color.g = position_rating/(br/2);
                rating_color.b = 1 - position_rating/(br/2);
            }

            geometry_msgs::Point gp0, gp1, gp2, gp3;
            gp0.x = p0.x; gp0.y = p0.y; gp0.z = p0.z + tfz;
            gp1.x = p1.x; gp1.y = p1.y; gp1.z = p1.z + tfz;
            gp2.x = p2.x; gp2.y = p2.y; gp2.z = p2.z + tfz;
            gp3.x = p3.x; gp3.y = p3.y; gp3.z = p3.z + tfz;


            marker_linelist.points.push_back(gp0);
            marker_linelist.points.push_back(gp1);
            marker_linelist.colors.push_back(rating_color);
            marker_linelist.points.push_back(gp1);
            marker_linelist.points.push_back(gp2);
            marker_linelist.colors.push_back(rating_color);
            marker_linelist.points.push_back(gp2);
            marker_linelist.points.push_back(gp3);
            marker_linelist.colors.push_back(rating_color);
            marker_linelist.points.push_back(gp3);
            marker_linelist.points.push_back(gp0);
            marker_linelist.colors.push_back(rating_color);
            marker_linelist.colors.push_back(rating_color);
            marker_linelist.colors.push_back(rating_color);
            marker_linelist.colors.push_back(rating_color);
            marker_linelist.colors.push_back(rating_color);
        }

        marker_array.markers.push_back(marker_linelist);
        pubPathRatingStates.publish(marker_array);
        marker_array.markers.clear();

    }

}



EnvironmentNAVXYTHETASTAB::EnvironmentNAVXYTHETASTAB()
{

    receivedWorldmodelPC=false;
    ros::NodeHandle nh_("~");

    pubTerrainModel =nh_.advertise<sensor_msgs::PointCloud2>("/hector/hector_sbpl_terrain_planner/cloud_input", 1);
    pubexpandedStates =nh_.advertise<sensor_msgs::PointCloud2>("/hector_sbpl_terrain_planner/expandedStates", 1);
    pubPathRatingStates =nh_.advertise<visualization_msgs::MarkerArray>("/hector_sbpl_terrain_planner/path_rating_rectangles", 1);
    pubRobotMarker =nh_.advertise<visualization_msgs::Marker>("/hector_sbpl_terrain_planner/robot_position", 1);
    expandedStatesCloud.clear();
    markers.markers.clear();
    markerID=0;
    flor_terrain_classifier::TerrainClassifierParams params(nh_);
    params.filter_mask = flor_terrain_classifier::FILTER_PASS_THROUGH | flor_terrain_classifier::FILTER_VOXEL_GRID | flor_terrain_classifier::FILTER_MLS_SMOOTH;
    ros::ServiceClient client = nh_.serviceClient<flor_terrain_classifier::TerrainModelService>("/flor/terrain_classifier/generate_terrain_model");
    flor_terrain_classifier::TerrainModelService srv;
    subTerrainModel= nh_.subscribe("/flor/terrain_classifier/cloud_input", 1000,  &EnvironmentNAVXYTHETASTAB::terrainModelCallback, this);
    subOctomap = nh_.subscribe("/hector_octomap_server/octomap_point_cloud_centers", 1000,  &EnvironmentNAVXYTHETASTAB::octomap_point_cloud_centers_Callback, this);
    subPath = nh_.subscribe("/drivepath", 1000,  &EnvironmentNAVXYTHETASTAB::pathCallback, this);
    subTF = nh_.subscribe("/tf", 1000, &EnvironmentNAVXYTHETASTAB::tfCallback, this);

    client.call(srv);
    ROS_INFO("called terrain_classifier/cloud_input service in EnvironmentNAVXYTHETASTAB constructor");
    //  tf_listener_.reset(new tf::TransformListener());
    //ros::Duration(0.1).sleep();
    int counter=0;
    while(!receivedWorldmodelPC)
    {
        sleep(1);
        counter++;
        ROS_INFO("[sbpl_terrain_planner: environment_navxytheta_stability_lat] Constructor spin %i - waiting for pointcloud to build environment.",counter);
        ros::spinOnce();
        //ros::Duration(0.1).sleep();
        client.call(srv);

    }

    ROS_INFO("environment built successfully");

}

EnvironmentNAVXYTHETASTAB::~EnvironmentNAVXYTHETASTAB()
{

}

pcl::PointXYZ rotatePoint(pcl::PointXYZ p, float degree /*radiants*/){
    return pcl::PointXYZ(cos(degree)*p.x - sin(degree)*p.y,
                         sin(degree)*p.x + cos(degree)*p.y,
                         p.z);
}

//Map is is the planner grid and world is the normal map
void EnvironmentNAVXYTHETASTAB::gridMapToMap(unsigned int mx, unsigned int my, double& wx, double& wy)
{
    int *size_x; int *size_y;
    int* num_thetas; double* startx; double* starty;
    double* starttheta; double* goalx; double* goaly; double* goaltheta; double* cellsize_m;
    double* nominalvel_mpersecs; double* timetoturn45degsinplace_secs;
    unsigned char* obsthresh; std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV;
    GetEnvParms(size_x,  size_y,   num_thetas,   startx,   starty, starttheta,   goalx,   goaly,   goaltheta,   cellsize_m,
                nominalvel_mpersecs,   timetoturn45degsinplace_secs, obsthresh,  motionprimitiveV);
    wx= -map_center_map.x+(mx+0.5)* *cellsize_m;
    wy= -map_center_map.y+(my+0.5)* *cellsize_m;
    //wx =  origin_x_ + (mx + 0.5) * resolution_;
    //wy = origin_y_ + (my + 0.5) * resolution_;
}

bool EnvironmentNAVXYTHETASTAB::mapToGridMap(double wx, double wy, unsigned int& mx, unsigned int& my)
{
    if(wx< - map_center_map.x || wy< - map_center_map.y)
        return false;

    int *size_x; int *size_y; int* num_thetas; double* startx; double* starty;
    double* starttheta; double* goalx; double* goaly; double* goaltheta; double* cellsize_m;
    double* nominalvel_mpersecs; double* timetoturn45degsinplace_secs;
    unsigned char* obsthresh; std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV;
    GetEnvParms(size_x,  size_y,   num_thetas,   startx,   starty, starttheta,   goalx,   goaly,   goaltheta,   cellsize_m,
                nominalvel_mpersecs,   timetoturn45degsinplace_secs, obsthresh,  motionprimitiveV);

    mx=(int)((wx + map_center_map.x)) / *cellsize_m;
    my=(int)((wy + map_center_map.y)) / *cellsize_m;
    return true;
}


void EnvironmentNAVXYTHETASTAB::UpdataData()
{
    ros::NodeHandle nh_("~");
    flor_terrain_classifier::TerrainClassifierParams params(nh_);
    params.filter_mask = flor_terrain_classifier::FILTER_PASS_THROUGH | flor_terrain_classifier::FILTER_VOXEL_GRID | flor_terrain_classifier::FILTER_MLS_SMOOTH;
    ros::ServiceClient client = nh_.serviceClient<flor_terrain_classifier::TerrainModelService>("/flor/terrain_classifier/generate_terrain_model");
    flor_terrain_classifier::TerrainModelService srv;
    //subTerrainModel = nh_.subscribe("/flor/terrain_classifier/cloud_input", 1000,  &EnvironmentNAVXYTHETASTAB::terrainModelCallback, this);
    client.call(srv);
    ros::spinOnce();
    ROS_INFO("Called Service");


}
int EnvironmentNAVXYTHETASTAB::GetActionCost(int SourceX, int SourceY, int SourceTheta,
                                             EnvNAVXYTHETALATAction_t* action)
{
    // ROS_INFO("GetActionCost: SourceX %i, SourceY %i, SourceTheta %i, actionEndTheta %i", SourceX, SourceY, SourceTheta, action->endtheta);
    int basecost = EnvironmentNAVXYTHETALAT::GetActionCost(SourceX, SourceY, SourceTheta, action);

    //float euclidicCost= sqrt(action->dX*action->dX+action->dY*action->dY);
    //float ratio=euclidicCost/(float)basecost;
    //ROS_INFO("Cost Ratio %f euclidic %f base %i",ratio, euclidicCost, basecost);
    //ROS_INFO("action: dX %i dY %i dT %i ",action->dX,action->dY,(int)action->endtheta-(int)action->starttheta);
    if (basecost >= INFINITECOST){
        ROS_WARN("basecost was >= INFINITECOST");
        return INFINITECOST;
    }
    int addcost = getAdditionalCost(SourceX, SourceY, SourceTheta, action);


  pcl::PointXYZI p;
    p.x=SourceX*0.05;
    p.y=SourceY*0.05;
    p.z=(double)addcost/100000.0;
    p.intensity=(double)addcost;
    expandedStatesCloud.push_back(p);
    sensor_msgs::PointCloud2 cloud_point_msg;
    pcl::toROSMsg(expandedStatesCloud, cloud_point_msg);
    cloud_point_msg.header.stamp = ros::Time::now();
    cloud_point_msg.header.frame_id = "map";
    pubexpandedStates.publish(cloud_point_msg);

    //  ROS_INFO("basecost:%i addcost:%i",basecost, addcost);

    if (addcost*basecost + basecost >= INFINITECOST)
        return INFINITECOST;

    return  addcost*basecost + basecost;
}

int EnvironmentNAVXYTHETASTAB::getAdditionalCost(int SourceX, int SourceY, int SourceTheta,
                                                 EnvNAVXYTHETALATAction_t* action)
{

    if (!IsValidCell(SourceX, SourceY)){
        ROS_WARN("sourceX, sourceY was not a valid cell");
        return INFINITECOST;
    }
    if (!IsValidCell(SourceX + action->dX, SourceY + action->dY)){
        ROS_WARN("sourceX, sourceY + delta was not a valid cell");
        return INFINITECOST;
    }

    pcl::PointXYZ checkPos((SourceX)*0.05f,(SourceY)*0.05f, 0.f);
    //Transformation
    /*   double checkpos_x;
    double checkpos_y;
    gridMapToMap(SourceX + action->dX, SourceY + action->dY, checkpos_x, checkpos_y);
    pcl::PointXYZ checkPos(checkpos_x, checkpos_y, 0.f);*/


    float positionRating;
    int invalidAxis;
    pcl::PointXYZ pc, p0, p1, p2, p3;

    //double time_start =ros::Time::now().toNSec();
    //  ROS_INFO("\n \n start computePositionRating with checkpos %f , %f, angle = %f", checkPos.x, checkPos.y, action->endtheta);

    bool positionRatingComputed = terrainModel.computePositionRating(checkPos, action->endtheta, pc, p0, p1, p2, p3, positionRating, invalidAxis);

    //double time_duration = (ros::Time::now().toNSec() - time_start)/1000;
    //ROS_INFO("time for CPR[mikrosec] = %i", (int)time_duration);

    if (!positionRatingComputed){
        ROS_INFO("no positionRatingComputed for x = %f, y = %f", checkPos.x, checkPos.y);
        return INFINITECOST;
    }

    if (positionRating < terrainModel.invalid_rating){
        ROS_INFO("invalid Rating %f",positionRating);
        return INFINITECOST;
    }
   // ROS_INFO("env_ : positionRating = %f, invalidAxis = %i ", positionRating, invalidAxis);
   // ROS_INFO("Position Rating %f /t %f x stable /t Orient %f /t Pos %f %f",positionRating,positionRating/flat_position_rating,action->endtheta,checkPos.x,checkPos.y);
   // ROS_INFO("Position Rating %f %f/t Orient %f /t Pos %f %f",positionRating,flat_position_rating,action->endtheta,checkPos.x,checkPos.y);

    if (positionRating >= flat_position_rating){
        return 0; // no additional costs
    }

    float rating_inv = flat_position_rating - positionRating; // high value is now bad -> 0 = perfect stable

   // float rating_inv = 1.0/positionRating-1.0/flat_position_rating;

    int addCost = (int)(pow(rating_inv, 2) * 20.0);
   // int addCost = (int)(rating_inv * 5.0);// (pow(rating_inv, 3) * 100.0 + invalidAxis * 60.0);


    return addCost;


}

bool EnvironmentNAVXYTHETASTAB::IsValidConfiguration(int X, int Y, int Theta)
{
    return true;
}
int EnvironmentNAVXYTHETASTAB::SetStart(double x, double y, double theta)
{
    // UpdataData();
    expandedStatesCloud.clear();
    EnvironmentNAVXYTHETALAT::SetStart(x,y,theta);
}

int EnvironmentNAVXYTHETASTAB::GetFromToHeuristic(int FromStateID, int ToStateID)
{

    ROS_INFO("GetFromToHeuristic");
    assert(0);
#if USE_HEUR==0
    return 0;
#endif
#if DEBUG
    if(FromStateID >= (int)StateID2CoordTable.size()
            || ToStateID >= (int)StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif
    //get X, Y for the state
    EnvNAVXYTHETALATHashEntry_t* FromHashEntry = StateID2CoordTable[FromStateID];
    EnvNAVXYTHETALATHashEntry_t* ToHashEntry = StateID2CoordTable[ToStateID];
    //TODO - check if one of the gridsearches already computed and then use it.
    return (int)(NAVXYTHETALAT_COSTMULT_MTOMM * EuclideanDistance_m(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X,
                                                                    ToHashEntry->Y) /
                 EnvNAVXYTHETALATCfg.nominalvel_mpersecs);
}
int EnvironmentNAVXYTHETASTAB::GetGoalHeuristic(int stateID)
{
//#if USE_HEUR==0
//    return 0;
//#endif
//#if DEBUG
//    if (stateID >= (int)StateID2CoordTable.size()) {
//        SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
//        throw new SBPL_Exception();
//    }
//#endif
//    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
//    //computes distances from start state that is grid2D, so it is EndX_c EndY_c
//    int h2D = grid2DsearchfromDsearchfromgoal->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y);
//    int hEuclid = (int)(NAVXYTHETALAT_COSTMULT_MTOMM * EuclideanDistance_m(HashEntry->X, HashEntry->Y,
//                                                                           EnvNAVXYTHETALATCfg.EndX_c,
//                                                                           EnvNAVXYTHETALATCfg.EndY_c));
//    //define this function if it is used in the planner (heuristic backward search would use it)
//    return (int)(((double)__max(h2D, hEuclid)) / EnvNAVXYTHETALATCfg.nominalvel_mpersecs);
    ROS_ERROR("GetGoalHeuristic!");
    assert(0);
    return 0;
}
int EnvironmentNAVXYTHETASTAB::GetStartHeuristic(int stateID)
{
#if USE_HEUR==0
    return 0;
#endif
#if DEBUG
    if (stateID >= (int)StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif
    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
//    int h2D = grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y);
    int hEuclid = (int)(NAVXYTHETALAT_COSTMULT_MTOMM * EuclideanDistance_m(EnvNAVXYTHETALATCfg.StartX_c,
                                                                           EnvNAVXYTHETALATCfg.StartY_c, HashEntry->X,
                                                                           HashEntry->Y));
    int hAngle = (int) (abs(EnvNAVXYTHETALATCfg.StartTheta-HashEntry->Theta)*40);
    //hAngle is only relevant if we are clos to the target
    if(hEuclid<1000)
        hAngle= (int)hAngle* (1000-hEuclid)/1000;
    else
        hAngle=0;
  //  ROS_INFO("hEuclid %i hAngle %i",hEuclid,hAngle);
    //define this function if it is used in the planner (heuristic backward search would use it)
    return (int)((double)(hEuclid+hAngle) / EnvNAVXYTHETALATCfg.nominalvel_mpersecs);
}
