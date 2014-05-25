//Implementation for new environment, analogous to https://github.com/sbpl/sbpl/tree/master/src/discrete_space_information

#include <hector_sbpl_terrain_planner/discrete_space_information/environment_navxytheta_stability_lat.h>
#include <cstdio>
#include <ctime>
#include <sbpl/utils/key.h>
#include <ros/ros.h>

using namespace std;

#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
#endif

static long int checks = 0;

//-----------------constructors/destructors-------------------------------

EnvironmentNAVXYTHETASTAB::EnvironmentNAVXYTHETASTAB()
{
    numofadditionalzlevs = 0; //by default there is only base level, no additional levels
    AddLevelFootprintPolygonV = NULL;
    AdditionalInfoinActionsV = NULL;
    AddLevelGrid2D = NULL;
    AddLevel_cost_possibly_circumscribed_thresh = NULL;
    AddLevel_cost_inscribed_thresh = NULL;
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

//returns true if cell is untraversable at all levels
bool EnvironmentNAVXYTHETASTAB::IsObstacle(int X, int Y)
{
    int levelind;

    if (EnvironmentNAVXYTHETALAT::IsObstacle(X, Y)) return true;

    //iterate through the additional levels
    for (levelind = 0; levelind < numofadditionalzlevs; levelind++) {
        if (AddLevelGrid2D[levelind][X][Y] >= EnvNAVXYTHETALATCfg.obsthresh) return true;
    }
    //otherwise the cell is obstacle-free at all cells
    return false;
}

//returns true if cell is untraversable in level # levelnum. If levelnum = -1, then it checks all levels
bool EnvironmentNAVXYTHETASTAB::IsObstacle(int X, int Y, int levind)
{
#if DEBUG
    if(levind >= numofadditionalzlevs)
    {
        SBPL_ERROR("ERROR: IsObstacle invoked at level %d\n", levind);
        SBPL_FPRINTF(fDeb, "ERROR: IsObstacle invoked at level %d\n", levind);
        return false;
    }
#endif

    return (AddLevelGrid2D[levind][X][Y] >= EnvNAVXYTHETALATCfg.obsthresh);
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

int EnvironmentNAVXYTHETASTAB::GetActionCost(int SourceX, int SourceY, int SourceTheta,
                                                EnvNAVXYTHETALATAction_t* action)
{
    int basecost = EnvironmentNAVXYTHETALAT::GetActionCost(SourceX, SourceY, SourceTheta, action);

    if (basecost >= INFINITECOST) return INFINITECOST;

    int addcost = basecost+GetActionCostacrossAddLevels(SourceX, SourceY, SourceTheta, action);

    //ROS_INFO("basecost:%i addcost:%i",basecost, addcost);

    return __max(basecost, addcost);
}

int EnvironmentNAVXYTHETASTAB::GetActionCostacrossAddLevels(int SourceX, int SourceY, int SourceTheta,
                                                               EnvNAVXYTHETALATAction_t* action)
{
    sbpl_2Dcell_t cell;
    sbpl_xy_theta_cell_t interm3Dcell;
    int i, levelind = -1;

    if (!IsValidCell(SourceX, SourceY)) return INFINITECOST;
    if (!IsValidCell(SourceX + action->dX, SourceY + action->dY)) return INFINITECOST;
    return 0;//abs(SourceY + action->dY)*500;


}

//---------------------------------------------------------------------

//------------debugging functions---------------------------------------------

//-----------------------------------------------------------------------------

//-----------interface with outside functions-----------------------------------

/*
initialization of additional levels. 0 is the original one. All additional ones will start with index 1
*/
bool EnvironmentNAVXYTHETASTAB::InitializeAdditionalLevels(int numofadditionalzlevs_in,
                                                              const vector<sbpl_2Dpt_t>* perimeterptsV,
                                                              unsigned char* cost_inscribed_thresh_in,
                                                              unsigned char* cost_possibly_circumscribed_thresh_in)
{
    int levelind = -1, xind = -1, yind = -1;
    sbpl_xy_theta_pt_t temppose;
    temppose.x = 0.0;
    temppose.y = 0.0;
    temppose.theta = 0.0;
    vector<sbpl_2Dcell_t> footprint;

    numofadditionalzlevs = numofadditionalzlevs_in;
    SBPL_PRINTF("Planning with additional z levels. Number of additional z levels = %d\n", numofadditionalzlevs);

    //allocate memory and set FootprintPolygons for additional levels
    AddLevelFootprintPolygonV = new vector<sbpl_2Dpt_t> [numofadditionalzlevs];
    for (levelind = 0; levelind < numofadditionalzlevs; levelind++) {
        AddLevelFootprintPolygonV[levelind] = perimeterptsV[levelind];
    }

    //print out the size of a footprint for each additional level
    for (levelind = 0; levelind < numofadditionalzlevs; levelind++) {
        footprint.clear();
        get_2d_footprint_cells(AddLevelFootprintPolygonV[levelind], &footprint, temppose,
                               EnvNAVXYTHETALATCfg.cellsize_m);
        SBPL_PRINTF("number of cells in footprint for additional level %d = %d\n", levelind,
                    (unsigned int)footprint.size());
    }

    //compute additional levels action info
    SBPL_PRINTF("pre-computing action data for additional levels:\n");
    AdditionalInfoinActionsV = new EnvNAVXYTHETASTABAddInfoAction_t*[EnvNAVXYTHETALATCfg.NumThetaDirs];
    for (int tind = 0; tind < EnvNAVXYTHETALATCfg.NumThetaDirs; tind++) {
        SBPL_PRINTF("pre-computing for angle %d out of %d angles\n", tind, EnvNAVXYTHETALATCfg.NumThetaDirs);

        //compute sourcepose
        sbpl_xy_theta_pt_t sourcepose;
        sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
        sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
        sourcepose.theta = DiscTheta2Cont(tind, EnvNAVXYTHETALATCfg.NumThetaDirs);

        AdditionalInfoinActionsV[tind] = new EnvNAVXYTHETASTABAddInfoAction_t[EnvNAVXYTHETALATCfg.actionwidth];

        //iterate over actions for each angle
        for (int aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++) {
            EnvNAVXYTHETALATAction_t* nav3daction = &EnvNAVXYTHETALATCfg.ActionsV[tind][aind];

            //initialize delta variables
            AdditionalInfoinActionsV[tind][aind].dX = nav3daction->dX;
            AdditionalInfoinActionsV[tind][aind].dY = nav3daction->dY;
            AdditionalInfoinActionsV[tind][aind].starttheta = tind;
            AdditionalInfoinActionsV[tind][aind].endtheta = nav3daction->endtheta;

            //finally, create the footprint for the action for each level
            AdditionalInfoinActionsV[tind][aind].intersectingcellsV = new vector<sbpl_2Dcell_t> [numofadditionalzlevs];
            for (levelind = 0; levelind < numofadditionalzlevs; levelind++) {
                get_2d_motion_cells(AddLevelFootprintPolygonV[levelind],
                                    EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV,
                                    &AdditionalInfoinActionsV[tind][aind].intersectingcellsV[levelind],
                                    EnvNAVXYTHETALATCfg.cellsize_m);
            }
        }
    }

    //create maps for additional levels and initialize to zeros (freespace)
    AddLevelGrid2D = new unsigned char**[numofadditionalzlevs];
    for (levelind = 0; levelind < numofadditionalzlevs; levelind++) {
        AddLevelGrid2D[levelind] = new unsigned char*[EnvNAVXYTHETALATCfg.EnvWidth_c];
        for (xind = 0; xind < EnvNAVXYTHETALATCfg.EnvWidth_c; xind++) {
            AddLevelGrid2D[levelind][xind] = new unsigned char[EnvNAVXYTHETALATCfg.EnvHeight_c];
            for (yind = 0; yind < EnvNAVXYTHETALATCfg.EnvHeight_c; yind++) {
                AddLevelGrid2D[levelind][xind][yind] = 0;
            }
        }
    }

    //create inscribed and circumscribed cost thresholds
    AddLevel_cost_possibly_circumscribed_thresh = new unsigned char[numofadditionalzlevs];
    AddLevel_cost_inscribed_thresh = new unsigned char[numofadditionalzlevs];
    for (levelind = 0; levelind < numofadditionalzlevs; levelind++) {
        AddLevel_cost_possibly_circumscribed_thresh[levelind] = cost_possibly_circumscribed_thresh_in[levelind];
        AddLevel_cost_inscribed_thresh[levelind] = cost_inscribed_thresh_in[levelind];
    }

    return true;
}

//set 2D map for the additional level levind
bool EnvironmentNAVXYTHETASTAB::Set2DMapforAddLev(const unsigned char* mapdata, int levind)
{
    int xind = -1, yind = -1;

    if (AddLevelGrid2D == NULL) {
        SBPL_ERROR("ERROR: failed to set2Dmap because the map was not allocated previously\n");
        return false;
    }

    for (xind = 0; xind < EnvNAVXYTHETALATCfg.EnvWidth_c; xind++) {
        for (yind = 0; yind < EnvNAVXYTHETALATCfg.EnvHeight_c; yind++) {
            AddLevelGrid2D[levind][xind][yind] = mapdata[xind + yind * EnvNAVXYTHETALATCfg.EnvWidth_c];
        }
    }

    return true;
}

//set 2D map for the additional level levind
//the version of Set2DMapforAddLev that takes newmap as 2D array instead of one linear array
bool EnvironmentNAVXYTHETASTAB::Set2DMapforAddLev(const unsigned char** NewGrid2D, int levind)
{
    int xind = -1, yind = -1;

    if (AddLevelGrid2D == NULL) {
        SBPL_ERROR("ERROR: failed to set2Dmap because the map was not allocated previously\n");
        return false;
    }

    for (xind = 0; xind < EnvNAVXYTHETALATCfg.EnvWidth_c; xind++) {
        for (yind = 0; yind < EnvNAVXYTHETALATCfg.EnvHeight_c; yind++) {
            AddLevelGrid2D[levind][xind][yind] = NewGrid2D[xind][yind];
        }
    }

    return true;
}

/*
update the traversability of a cell<x,y> in addtional level zlev (this is not to update basic level)
*/
bool EnvironmentNAVXYTHETASTAB::UpdateCostinAddLev(int x, int y, unsigned char newcost, int zlev)
{
#if DEBUG
    //SBPL_FPRINTF(fDeb, "Cost updated for cell %d %d at level %d from old cost=%d to new cost=%d\n", x, y, zlev, AddLevelGrid2D[zlev][x][y], newcost);
#endif

    AddLevelGrid2D[zlev][x][y] = newcost;

    //no need to update heuristics because at this point it is computed solely based on the basic level

    return true;
}

//------------------------------------------------------------------------------
