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
           ROS_INFO("cloud was just initialized. size = %i", cloud.size());
           ROS_INFO("cloud in terrainModel size %i", terrainModel.cloud_processed.size());
           ROS_INFO("cloudPTR in terrainModel size %i", terrainModel.cloud_processed_Ptr->size());
           sleep(1);
        }
        else{
            ROS_INFO("entered Callback, world model was received before");
            // GEHT NICHT  : terrainModel.cloud_processed_Ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr (&terrainModel.cloud_processed);
            ROS_INFO("cloud_processed_Ptr in terrainModel: size = %i", terrainModel.cloud_processed_Ptr->size());
            ROS_INFO("cloud size (von env aus) %i", terrainModel.cloud_processed.size());
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
        ROS_INFO("End dur");
        client.call(srv);

    }

    ROS_INFO(".called terrain_classifier/cloud_input service in EnvironmentNAVXYTHETASTAB constructor END");

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
    int basecost = EnvironmentNAVXYTHETALAT::GetActionCost(SourceX, SourceY, SourceTheta, action);

    if (basecost >= INFINITECOST) return INFINITECOST;

    int addcost = getAdditionalCost(SourceX, SourceY, SourceTheta, action);

    ROS_INFO("basecost:%i addcost:%i",basecost, addcost);

    return  addcost + basecost;
}

int EnvironmentNAVXYTHETASTAB::getAdditionalCost(int SourceX, int SourceY, int SourceTheta,
                                                               EnvNAVXYTHETALATAction_t* action)
{
    //sbpl_2Dcell_t cell;
  //  sbpl_xy_theta_cell_t interm3Dcell;
  //  int i, levelind = -1;


    ROS_INFO("getAddCost");

    if (!IsValidCell(SourceX, SourceY)) return INFINITECOST;
    if (!IsValidCell(SourceX + action->dX, SourceY + action->dY)) return INFINITECOST;

    pcl::PointXYZ checkPos((SourceX+ action->dX)*0.01f,(SourceY+ action->dY)*0.01f, 0.f);
    //Transformation
 /*   double checkpos_x;
    double checkpos_y;
    gridMapToMap(SourceX + action->dX, SourceY + action->dY, checkpos_x, checkpos_y);
    pcl::PointXYZ checkPos(checkpos_x, checkpos_y, 0.f);*/


    float positionRating;
    int invalidAxis;

    ROS_INFO("computePositionRating checkpos %f , %f", checkPos.x, checkPos.y);
    bool positionRatingComputed = terrainModel.computePositionRating(checkPos, action->endtheta, positionRating, invalidAxis);

    ROS_INFO("computePosRating End");
    if (!positionRatingComputed){
        return INFINITECOST;
    }

    if (positionRating > 1.0f){
        return INFINITECOST;
    }

    int addCost = (int) (positionRating * 5000.0 + invalidAxis * 5000.0);
    ROS_INFO("addcost = %i, positionRating = %f, invalidAxis = %i", addCost, positionRating, invalidAxis);

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

