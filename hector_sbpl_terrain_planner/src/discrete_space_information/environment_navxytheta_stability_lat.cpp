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
        ROS_INFO("entered callback");

        if(!receivedWorldmodelPC)
        {
           receivedWorldmodelPC=true;
           pcl::PCLPointCloud2 pcl_pc;
           pcl_conversions::toPCL(msg, pcl_pc);
           pcl::PointCloud<pcl::PointXYZ> cloud;
           pcl::fromPCLPointCloud2(pcl_pc, cloud);
           terrainModel = hector_terrain_model::TerrainModel(cloud);
        }
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
    ROS_INFO("called service in constructor");
    int counter=0;
    while(!receivedWorldmodelPC)
    {
        counter++;
        ROS_INFO("Constructor spin %i",counter);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        client.call(srv);
    }

}

EnvironmentNAVXYTHETASTAB::~EnvironmentNAVXYTHETASTAB()
{

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

    int addcost = basecost+getAdditionalCost(SourceX, SourceY, SourceTheta, action);

    ROS_INFO("basecost:%i addcost:%i",basecost, addcost);

    return  addcost;
}

int EnvironmentNAVXYTHETASTAB::getAdditionalCost(int SourceX, int SourceY, int SourceTheta,
                                                               EnvNAVXYTHETALATAction_t* action)
{
    sbpl_2Dcell_t cell;
    sbpl_xy_theta_cell_t interm3Dcell;
    int i, levelind = -1;

    pcl::PointXYZ checkPos((SourceX+ action->dX)*0.01f,(SourceY+ action->dY)*0.01f-2.f,0.f);


    float addCost=0.f;//  terrainModel.computePositionRating(checkPos, action->endtheta);
  //  ROS_INFO("cost %f", addCost);

    if (!IsValidCell(SourceX, SourceY)) return INFINITECOST;
    if (!IsValidCell(SourceX + action->dX, SourceY + action->dY)) return INFINITECOST;
    return addCost;


}
bool EnvironmentNAVXYTHETASTAB::IsValidConfiguration(int X, int Y, int Theta)
{
    true;
}
 int EnvironmentNAVXYTHETASTAB::SetStart(double x, double y, double theta)
{
   // UpdataData();
    EnvironmentNAVXYTHETALAT::SetStart(x,y,theta);
}

/** bool EnvironmentNAVXYTHETASTAB::IsWithinMapCell(int X,int Y)
 {
    return true;
 }**/
/**
 void EnvironmentNAVXYTHETASTAB::SetConfiguration(int width, int height,
                                          const unsigned char* mapdata,
                                         int startx, int starty,
                                          int goalx, int goaly) {
    EnvNAV2DCfg.EnvWidth_c = width;
    EnvNAV2DCfg.EnvHeight_c = height;
    EnvNAV2DCfg.StartX_c = startx;
    EnvNAV2DCfg.StartY_c = starty;
    int x;



    EnvNAV2DCfg.EndX_c = goalx;
    EnvNAV2DCfg.EndY_c = goaly;

    //allocate the 2D environment
    EnvNAV2DCfg.Grid2D = new unsigned char* [EnvNAV2DCfg.EnvWidth_c];
    for (x = 0; x < EnvNAV2DCfg.EnvWidth_c; x++) {
      EnvNAV2DCfg.Grid2D[x] = new unsigned char [EnvNAV2DCfg.EnvHeight_c];
    }


    //environment:
    if (0 == mapdata) {
      for (int y = 0; y < EnvNAV2DCfg.EnvHeight_c; y++) {
        for (int x = 0; x < EnvNAV2DCfg.EnvWidth_c; x++) {
          EnvNAV2DCfg.Grid2D[x][y] = 0;
        }
      }
    }
    else {
      for (int y = 0; y < EnvNAV2DCfg.EnvHeight_c; y++) {
        for (int x = 0; x < EnvNAV2DCfg.EnvWidth_c; x++) {
          unsigned char cval = mapdata[x+y*width];
          EnvNAV2DCfg.Grid2D[x][y] = cval;
        }
      }
    }

 }**/
