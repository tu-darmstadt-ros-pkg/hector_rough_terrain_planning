//Header for new environment, analogous to https://github.com/sbpl/sbpl/tree/master/src/include/sbpl

#ifndef __ENVIRONMENT_NAVXYTHETA_STABILITY_LAT_H_
#define __ENVIRONMENT_NAVXYTHETA_STABILITY_LAT_H_


#include <vector>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#include <sbpl/sbpl_exception.h>
#include <sbpl/utils/utils.h>
#include <flor_terrain_classifier/terrain_model.h>
#include <sensor_msgs/PointCloud2.h>

// these structures contain footprints for the additional levels
// each of these structures corresponds to one of the EnvNAVXYTHETALATAction_t structures
typedef struct
{
    char starttheta; // should be equal to the corresponding EnvNAVXYTHETALATAction_t structure
    char dX; // should be equal to the corresponding EnvNAVXYTHETALATAction_t structure
    char dY; // should be equal to the corresponding EnvNAVXYTHETALATAction_t structure
    char endtheta; // should be equal to the corresponding EnvNAVXYTHETALATAction_t structure
    std::vector<sbpl_2Dcell_t>* intersectingcellsV; // one footprint per additional level
} EnvNAVXYTHETASTABAddInfoAction_t;

/**
* \brief This is x,y,theta lattice planning but with multiple levels in z.
* In other words, it is for doing collision checking in 3D (x,y,z). The z
* level is split into numofzlevs levels. If numofzlevs = 1, then it
* defaults to the original x,y,theta lattice planning defined in
* EnvironmentNAVXYTHETALAT. Otherwise, it uses numofzlevs footprints of the
* robot and corresponding costmaps. It assumes that they correspond to each
* other and are projections of the robot and corresponding z regions of the
* 3D map.
*/
class EnvironmentNAVXYTHETASTAB : public EnvironmentNAVXYTHETALAT
{
public:


    /**
* \brief incremental planning not supported
*/
    virtual void GetPredsofChangedEdges(std::vector<nav2dcell_t> const * changedcellsV,
                                        std::vector<int> *preds_of_changededgesIDV)
    {
        SBPL_ERROR("ERROR: GetPredsofChangedEdges function not supported\n");
        throw new SBPL_Exception();
    }

    /**
* \brief incremental planning not supported
*/
    virtual void GetSuccsofChangedEdges(std::vector<nav2dcell_t> const * changedcellsV,
                                        std::vector<int> *succs_of_changededgesIDV)
    {
        SBPL_ERROR("ERROR: GetSuccsofChangedEdges function not supported\n");
        throw new SBPL_Exception();
    }

    /**
* returns true if cell is traversable and within map limits - it checks against all levels including the base one
*/
    bool IsValidCell(int X, int Y);

    /**
* returns true if cell is traversable and within map limits for a particular level
*/
    bool IsValidCell(int X, int Y, int levind);

    /**
* returns true if cell is untraversable at any level
*/
    bool IsObstacle(int x, int y);

    /**
* returns true if cell is untraversable at level levelnum.
*/
    bool IsObstacle(int x, int y, int levind);

    /**
* \brief returns false if robot intersects obstacles or lies outside of the map.
* Note this is pretty expensive operation since it computes the footprint
* of the robot based on its x,y,theta
*/
    bool IsValidConfiguration(int X, int Y, int Theta);

    /**
* \brief returns the maximum over all levels of the cost corresponding to the cell <x,y>
*/
    unsigned char GetMapCost(int X, int Y);

    /**
* \brief returns the cost corresponding to the cell <x,y> at level levind
*/
    unsigned char GetMapCost(int X, int Y, int levind);

    EnvironmentNAVXYTHETASTAB();
    ~EnvironmentNAVXYTHETASTAB();


    void terrainModelCallback(const sensor_msgs::PointCloud2 msg);

protected:

    hector_terrain_model::TerrainModel terrainModel;
    /**
* \brief number of additional levels. If it is 0, then there is only one level - base level
*/
    int numofadditionalzlevs;

    /**
* \brief footprints for the additional levels
*/
    std::vector<sbpl_2Dpt_t>* AddLevelFootprintPolygonV;

    /**
* \brief array of additional info in actions,
* AdditionalInfoinActionsV[i][j] - jth action for sourcetheta = i
* basically, each Additional info structure will contain numofadditionalzlevs additional intersecting
* cells vector<sbpl_2Dcell_t> intersectingcellsV
*/
    EnvNAVXYTHETASTABAddInfoAction_t** AdditionalInfoinActionsV;

    /**
* \brief 2D maps for additional levels.
* AddLevelGrid2D[lind][x][y] refers to <x,y> cell on the additional level lind
*/
    unsigned char*** AddLevelGrid2D;

    /**
* \brief inscribed cost thresholds for additional levels
* see environment_navxythetalat.h file for the explanation of this threshold
*/
    unsigned char* AddLevel_cost_inscribed_thresh;
    /**
* \brief possibly_circumscribed cost thresholds for additional levels
* see environment_navxythetalat.h file for the explanation of this threshold
*/
    unsigned char* AddLevel_cost_possibly_circumscribed_thresh;

    virtual int GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action);

    virtual int GetActionCostacrossAddLevels(int SourceX, int SourceY, int SourceTheta,
                                             EnvNAVXYTHETALATAction_t* action);


};
#endif
