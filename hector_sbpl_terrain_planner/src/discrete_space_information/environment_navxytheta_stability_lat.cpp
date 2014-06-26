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
        ROS_INFO("entered callback");
        ROS_INFO("entered callback");
        ROS_INFO("entered callback");
        ROS_INFO("entered callback");
        ROS_INFO("entered callback");
        ROS_INFO("entered callback");
        ROS_INFO("entered callback");
        ROS_INFO("entered callback");
        ROS_INFO("entered callback");
        ROS_INFO("entered callback");
       pcl::PCLPointCloud2 pcl_pc;
       pcl_conversions::toPCL(msg, pcl_pc);
       pcl::PointCloud<pcl::PointXYZ> cloud;
       pcl::fromPCLPointCloud2(pcl_pc, cloud);
       terrainModel = hector_terrain_model::TerrainModel(cloud);
  }


EnvironmentNAVXYTHETASTAB::EnvironmentNAVXYTHETASTAB()
{
    numofadditionalzlevs = 0; //by default there is only base level, no additional levels
    AddLevelFootprintPolygonV = NULL;
    AdditionalInfoinActionsV = NULL;
    AddLevelGrid2D = NULL;
    AddLevel_cost_possibly_circumscribed_thresh = NULL;
    AddLevel_cost_inscribed_thresh = NULL;
    ros::NodeHandle nh_("~");
    flor_terrain_classifier::TerrainClassifierParams params(nh_);
    params.filter_mask = flor_terrain_classifier::FILTER_PASS_THROUGH | flor_terrain_classifier::FILTER_VOXEL_GRID | flor_terrain_classifier::FILTER_MLS_SMOOTH;
    ros::ServiceClient client = nh_.serviceClient<flor_terrain_classifier::TerrainModelService>("/flor/terrain_classifier/generate_terrain_model");
    flor_terrain_classifier::TerrainModelService srv;
    subTerrainModel= nh_.subscribe("/flor/terrain_classifier/cloud_input", 1000,  &EnvironmentNAVXYTHETASTAB::terrainModelCallback, this);
    client.call(srv);
    ROS_INFO("called service in constructor");
    ros::spinOnce();
}

EnvironmentNAVXYTHETASTAB::~EnvironmentNAVXYTHETASTAB()
{
    if (AddLevelFootprintPolygonV != NULL) {
        delete[] AddLevelFootprintPolygonV;
        AddLevelFootprintPolygonV = NULL;
    }

    if (AdditionalInfoinActionsV != NULL) {
        for (int tind = 0; tind < EnvNAVXYTHETALATCfg.NumThetaDirs; tind++) {
            for (int aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++) {
                delete[] AdditionalInfoinActionsV[tind][aind].intersectingcellsV;
            }
            delete[] AdditionalInfoinActionsV[tind];
        }
        delete[] AdditionalInfoinActionsV;
        AdditionalInfoinActionsV = NULL;
    }

    if (AddLevelGrid2D != NULL) {
        for (int levelind = 0; levelind < numofadditionalzlevs; levelind++) {
            for (int xind = 0; xind < EnvNAVXYTHETALATCfg.EnvWidth_c; xind++) {
                delete[] AddLevelGrid2D[levelind][xind];
            }
            delete[] AddLevelGrid2D[levelind];
        }
        delete[] AddLevelGrid2D;
        AddLevelGrid2D = NULL;
    }

    if (AddLevel_cost_possibly_circumscribed_thresh != NULL) {
        delete[] AddLevel_cost_possibly_circumscribed_thresh;
        AddLevel_cost_possibly_circumscribed_thresh = NULL;
    }

    if (AddLevel_cost_inscribed_thresh != NULL) {
        delete[] AddLevel_cost_inscribed_thresh;
        AddLevel_cost_inscribed_thresh = NULL;
    }

    //reset the number of additional levels
    numofadditionalzlevs = 0;
}

//---------------------------------------------------------------------

//-------------------problem specific and local functions---------------------

//returns true if cell is traversable and within map limits - it checks against all levels including the base one
bool EnvironmentNAVXYTHETASTAB::IsValidCell(int X, int Y)
{
    int levelind;

    if (!EnvironmentNAVXYTHETALAT::IsValidCell(X, Y)) return false;

    //iterate through the additional levels
    for (levelind = 0; levelind < numofadditionalzlevs; levelind++) {
        if (AddLevelGrid2D[levelind][X][Y] >= EnvNAVXYTHETALATCfg.obsthresh) return false;
    }
    //otherwise the cell is valid at all levels
    return true;
}

// returns true if cell is traversable and within map limits for a particular level
bool EnvironmentNAVXYTHETASTAB::IsValidCell(int X, int Y, int levind)
{
    return (X >= 0 && X < EnvNAVXYTHETALATCfg.EnvWidth_c && Y >= 0 && Y < EnvNAVXYTHETALATCfg.EnvHeight_c && levind <
            numofadditionalzlevs && AddLevelGrid2D[levind][X][Y] < EnvNAVXYTHETALATCfg.obsthresh);
}


// returns the maximum over all levels of the cost corresponding to the cell <x,y>
unsigned char EnvironmentNAVXYTHETASTAB::GetMapCost(int X, int Y)
{
    unsigned char mapcost = EnvNAVXYTHETALATCfg.Grid2D[X][Y];

    for (int levind = 0; levind < numofadditionalzlevs; levind++) {
        mapcost = __max(mapcost, AddLevelGrid2D[levind][X][Y]);
    }

    return mapcost;
}

// returns the cost corresponding to the cell <x,y> at level levind
unsigned char EnvironmentNAVXYTHETASTAB::GetMapCost(int X, int Y, int levind)
{
#if DEBUG
    if(levind >= numofadditionalzlevs)
    {
        SBPL_ERROR("ERROR: GetMapCost invoked at level %d\n", levind);
        SBPL_FPRINTF(fDeb, "ERROR: GetMapCost invoked at level %d\n", levind);
        return false;
    }
#endif

    return AddLevelGrid2D[levind][X][Y];
}

//returns false if robot intersects obstacles or lies outside of the map.
bool EnvironmentNAVXYTHETASTAB::IsValidConfiguration(int X, int Y, int Theta)
{
    //check the base footprint first
    if (!EnvironmentNAVXYTHETALAT::IsValidConfiguration(X, Y, Theta)) return false;

    //check the remaining levels now
    vector<sbpl_2Dcell_t> footprint;
    sbpl_xy_theta_pt_t pose;

    //compute continuous pose
    pose.x = DISCXY2CONT(X, EnvNAVXYTHETALATCfg.cellsize_m);
    pose.y = DISCXY2CONT(Y, EnvNAVXYTHETALATCfg.cellsize_m);
    pose.theta = DiscTheta2Cont(Theta, EnvNAVXYTHETALATCfg.NumThetaDirs);

    //iterate over additional levels
    for (int levind = 0; levind < numofadditionalzlevs; levind++) {

        //compute footprint cells
        footprint.clear();
        get_2d_footprint_cells(AddLevelFootprintPolygonV[levind], &footprint, pose, EnvNAVXYTHETALATCfg.cellsize_m);

        //iterate over all footprint cells
        for (int find = 0; find < (int)footprint.size(); find++) {
            int x = footprint.at(find).x;
            int y = footprint.at(find).y;

            if (x < 0 || x >= EnvNAVXYTHETALATCfg.EnvWidth_c || y < 0 || y >= EnvNAVXYTHETALATCfg.EnvHeight_c
                || AddLevelGrid2D[levind][x][y] >= EnvNAVXYTHETALATCfg.obsthresh) {
                return false;
            }
        }
    }

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

}
int EnvironmentNAVXYTHETASTAB::GetActionCost(int SourceX, int SourceY, int SourceTheta,
                                                EnvNAVXYTHETALATAction_t* action)
{
    int basecost = EnvironmentNAVXYTHETALAT::GetActionCost(SourceX, SourceY, SourceTheta, action);

    if (basecost >= INFINITECOST) return INFINITECOST;

    int addcost = basecost+GetActionCostacrossAddLevels(SourceX, SourceY, SourceTheta, action);

    ROS_INFO("basecost:%i addcost:%i",basecost, addcost);

    return  addcost;
}

int EnvironmentNAVXYTHETASTAB::GetActionCostacrossAddLevels(int SourceX, int SourceY, int SourceTheta,
                                                               EnvNAVXYTHETALATAction_t* action)
{
    sbpl_2Dcell_t cell;
    sbpl_xy_theta_cell_t interm3Dcell;
    int i, levelind = -1;

    pcl::PointXYZ checkPos((SourceX+ action->dX)*0.01f,(SourceY+ action->dY)*0.01f-2.f,0.f);


    float addCost=  terrainModel.computePositionRating(checkPos, action->endtheta);
    ROS_INFO("cost %f", addCost);

    if (!IsValidCell(SourceX, SourceY)) return INFINITECOST;
    if (!IsValidCell(SourceX + action->dX, SourceY + action->dY)) return INFINITECOST;
    return addCost;


}

 int EnvironmentNAVXYTHETASTAB::SetStart(double x, double y, double theta)
{
    //UpdataData();
    EnvironmentNAVXYTHETALAT::SetStart(x,y,theta);
}
