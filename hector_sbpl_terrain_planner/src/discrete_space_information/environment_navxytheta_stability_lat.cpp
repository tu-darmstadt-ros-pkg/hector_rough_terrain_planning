//Implementation for new environment, analogous to https://github.com/sbpl/sbpl/tree/master/src/discrete_space_information

#include <hector_sbpl_terrain_planner/discrete_space_information/environment_navxytheta_stability_lat.h>
#include <cstdio>
#include <ctime>
#include <sbpl/utils/key.h>
//#include <geometry_msgs/pos>
//#include <flor_terrain_classifier/terrain_classifier.h>
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>

#include <flor_terrain_classifier/TestModelService.h>
#include <flor_terrain_classifier/TestModel.h>
#include <flor_terrain_classifier/TerrainModel.h>
#include <flor_terrain_classifier/TerrainModelService.h>
#include <flor_terrain_classifier/terrain_classifier.h>
#include <visualization_msgs/Marker.h>
using namespace std;

#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
#endif

static long int checks = 0;

//-----------------constructors/destructors-------------------------------

void EnvironmentNAVXYTHETASTAB::terrainModelCallback(const sensor_msgs::PointCloud2 msg)
  {
        ROS_INFO("entered callback...");


        if(!receivedWorldmodelPC)
        {
           receivedWorldmodelPC=true;
           pcl::PCLPointCloud2 pcl_pc;
           pcl_conversions::toPCL(msg, pcl_pc);
           pcl::PointCloud<pcl::PointXYZ> cloud;
           pcl::fromPCLPointCloud2(pcl_pc, cloud);
           terrainModel = hector_terrain_model::TerrainModel(cloud);
           ROS_INFO("cloudPTR in terrainModel size %i", terrainModel.cloud_processed_Ptr->size());
           sleep(1);
           sensor_msgs::PointCloud2 cloud_point_msg;
           pcl::toROSMsg(cloud, cloud_point_msg);
           cloud_point_msg.header.stamp = ros::Time::now();
           cloud_point_msg.header.frame_id = "map";
           terrainModelPublisher.publish(cloud_point_msg);
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

EnvironmentNAVXYTHETASTAB::EnvironmentNAVXYTHETASTAB()
{

    receivedWorldmodelPC=false;
    ros::NodeHandle nh_("~");

    terrainModelPublisher =nh_.advertise<sensor_msgs::PointCloud2>("/hector/hector_sbpl_terrain_planner/cloud_input", 1);
    expandedStatesPublisher =nh_.advertise<sensor_msgs::PointCloud2>("/hector/hector_sbpl_terrain_planner/expandedStates", 1);
    expandedStatesCloud.clear();
    markers.markers.clear();
    markerID=0;
    flor_terrain_classifier::TerrainClassifierParams params(nh_);
    params.filter_mask = flor_terrain_classifier::FILTER_PASS_THROUGH | flor_terrain_classifier::FILTER_VOXEL_GRID | flor_terrain_classifier::FILTER_MLS_SMOOTH;
    ros::ServiceClient client = nh_.serviceClient<flor_terrain_classifier::TerrainModelService>("/flor/terrain_classifier/generate_terrain_model");
    flor_terrain_classifier::TerrainModelService srv;
    subTerrainModel= nh_.subscribe("/flor/terrain_classifier/cloud_input", 1000,  &EnvironmentNAVXYTHETASTAB::terrainModelCallback, this);
    client.call(srv);
    ROS_INFO("called terrain_classifier/cloud_input service in EnvironmentNAVXYTHETASTAB constructor");
  //  tf_listener_.reset(new tf::TransformListener());
    //ros::Duration(0.1).sleep();
    int counter=0;
    while(!receivedWorldmodelPC)
    {
        sleep(1);
        counter++;
        ROS_INFO("Constructor spin %i",counter);
        ros::spinOnce();
        //ros::Duration(0.1).sleep();
        client.call(srv);

    }

    //ROS_INFO("called terrain_classifier/cloud_input service in EnvironmentNAVXYTHETASTAB constructor END");

}

EnvironmentNAVXYTHETASTAB::~EnvironmentNAVXYTHETASTAB()
{

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

//    if (wx < origin_x_ || wy < origin_y_)
  //      return false;

  //  mx = (int)((wx - origin_x_) / resolution_);
  //  my = (int)((wy - origin_y_) / resolution_);

 //   if (mx < size_x_ && my < size_y_)
  //      return true;

  //  return false;
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
    ROS_INFO("GetActionCost: SourceX %i, SourceY %i, SourceTheta %i, actionEndTheta %c", SourceX, SourceY, SourceTheta, action->endtheta);
    int basecost = EnvironmentNAVXYTHETALAT::GetActionCost(SourceX, SourceY, SourceTheta, action);

    if (basecost >= INFINITECOST) return INFINITECOST;

    int addcost = getAdditionalCost(SourceX, SourceY, SourceTheta, action);

    float robotSize=0.3;
    for(unsigned int i=0; i<10; ++i)
    {
        for(unsigned int j=0; j<10; ++j)
        {
            float costInt=addcost;
            pcl::PointXYZI p;

            p.x=SourceX*0.05+i*robotSize/20.0;
            p.y=SourceY*0.05+j*robotSize/20.0;
            p.z=0.0;
            p.intensity=(double)addcost;
            expandedStatesCloud.push_back(p);
        }
    }

    sensor_msgs::PointCloud2 cloud_point_msg;
    pcl::toROSMsg(expandedStatesCloud, cloud_point_msg);
    cloud_point_msg.header.stamp = ros::Time::now();
    cloud_point_msg.header.frame_id = "map";
    expandedStatesPublisher.publish(cloud_point_msg);
/**

     uint32_t shape = visualization_msgs::Marker::CUBE;
     visualization_msgs::Marker marker;
     // Set the frame ID and timestamp.  See the TF tutorials for information on these.
     marker.header.frame_id = "/map";
     marker.header.stamp = ros::Time::now();

     // Set the namespace and id for this marker.  This serves to create a unique ID
     // Any marker sent with the same namespace and id will overwrite the old one
     marker.ns = "basic_shapes"+boost::lexical_cast<std::string>(markerID);
     marker.id = markerID;
     markerID++;


         // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
         marker.type = shape;

         // Set the marker action.  Options are ADD and DELETE
         marker.action = visualization_msgs::Marker::ADD;

         // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
         marker.pose.position.x = SourceX*0.05;
         marker.pose.position.y = SourceY*0.05;
         marker.pose.position.z = 0;
         marker.pose.orientation.x = 0.0;
         marker.pose.orientation.y = 0.0;
         marker.pose.orientation.z = 0.0;
         marker.pose.orientation.w = 1.0;

         // Set the scale of the marker -- 1x1x1 here means 1m on a side
         marker.scale.x = 0.10;
         marker.scale.y = 0.10;
         marker.scale.z = 0.010;

         // Set the color -- be sure to set alpha to something non-zero!
         if(addcost>10000){
             marker.color.r = 1.0f;
         marker.color.g = 0.0f;
         marker.color.b = 0.0f;}
         else
            {
             marker.color.r = 0.0f;
             marker.color.g = abs(1-(addcost/100.0));
             marker.color.b = abs((addcost/100.0));
         }
         marker.color.a = 1.0;

         marker.lifetime = ros::Duration();

         markers.markers.push_back(marker);

         // Publish the marker
        expandedStatesPublisher.publish(markers);**/

    ROS_INFO("basecost:%i addcost:%i",basecost, addcost);

    return  addcost + basecost;
}

int EnvironmentNAVXYTHETASTAB::getAdditionalCost(int SourceX, int SourceY, int SourceTheta,
                                                               EnvNAVXYTHETALATAction_t* action)
{

    ROS_INFO("actionDebug: action char: x = %i ,y= %i ,endtheta = %i ", action->dX, action->dY, action->endtheta);
    //sbpl_2Dcell_t cell;
  //  sbpl_xy_theta_cell_t interm3Dcell;
  //  int i, levelind = -1;

    if (!IsValidCell(SourceX, SourceY)) return INFINITECOST;
    if (!IsValidCell(SourceX + action->dX, SourceY + action->dY)) return INFINITECOST;

    pcl::PointXYZ checkPos((SourceX)*0.05f,(SourceY)*0.05f, 0.f);
    //Transformation
 /*   double checkpos_x;
    double checkpos_y;
    gridMapToMap(SourceX + action->dX, SourceY + action->dY, checkpos_x, checkpos_y);
    pcl::PointXYZ checkPos(checkpos_x, checkpos_y, 0.f);*/


    float positionRating;
    int invalidAxis;


    double time_start =ros::Time::now().toNSec();
    ROS_INFO("\n start computePositionRating with checkpos %f , %f, angle = %c", checkPos.x, checkPos.y, action->endtheta);
    bool positionRatingComputed = terrainModel.computePositionRating(checkPos, action->endtheta, positionRating, invalidAxis);
    double time_duration = (ros::Time::now().toNSec() - time_start)/1000;
    ROS_INFO("time for CPR[mikrosec] = %i", (int)time_duration);

    if (!positionRatingComputed){
        return INFINITECOST;
    }

    if (positionRating < terrainModel.invalid_rating){
        return INFINITECOST;
    }


    ROS_INFO("env_ : positionRating = %f, invalidAxis = %i ", positionRating, invalidAxis);

    positionRating = pow((1/positionRating),3); // self invented -> good?
    int addCost = (int) (positionRating * 100.0 + invalidAxis * 75.0);

    return addCost;


}

bool EnvironmentNAVXYTHETASTAB::IsValidConfiguration(int X, int Y, int Theta)
{
    return true;
}
 int EnvironmentNAVXYTHETASTAB::SetStart(double x, double y, double theta)
{
   // UpdataData();
    EnvironmentNAVXYTHETALAT::SetStart(x,y,theta);
}

