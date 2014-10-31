//Header for new environment, analogous to https://github.com/sbpl/sbpl/tree/master/src/include/sbpl

#ifndef __ENVIRONMENT_NAVXYTHETA_STABILITY_LAT_H_
#define __ENVIRONMENT_NAVXYTHETA_STABILITY_LAT_H_


#include <vector>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#include <sbpl/sbpl_exception.h>
#include <sbpl/utils/utils.h>
#include <flor_terrain_classifier/terrain_model.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/MarkerArray.h>

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
     * \brief see comments on the same function in the parent class
     */
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int GetGoalHeuristic(int stateID);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int GetStartHeuristic(int stateID);

virtual bool IsValidConfiguration(int X, int Y, int Theta);

    EnvironmentNAVXYTHETASTAB();
    ~EnvironmentNAVXYTHETASTAB();

    // bool IsWithinMapCell(int X,int Y);

    void octomap_point_cloud_centers_Callback(const sensor_msgs::PointCloud2 msg);
    void pathCallback(const nav_msgs::Path msg);
    void tfCallback(const tf2_msgs::TFMessage &msg);
    void terrainModelCallback(const sensor_msgs::PointCloud2 msg);
    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map);
    void UpdataData();
    virtual int SetStart(double x, double y, double theta);


    void gridMapToMap(unsigned int mx, unsigned int my, double& wx, double& wy);
    bool mapToGridMap(double wx, double wy, unsigned int& mx, unsigned int& my);

protected:

    hector_terrain_model::TerrainModel terrainModel;


    virtual int GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action);

    virtual int getAdditionalCost(int SourceX, int SourceY, int SourceTheta,
                                             EnvNAVXYTHETALATAction_t* action);


   // void mapToTf(double mx, double my, double& tx, double& ty);
   // void tfToMap(double tx, double ty, double& mx, double& my);

    ros::Subscriber subTerrainModel;
    ros::Subscriber subOctomap;
    ros::Subscriber subPath;
    ros::Subscriber subTF;

    bool receivedWorldmodelPC;

    ros::Time t_lastMapPos_;
    ros::Subscriber map_subscriber_;
    std::vector<geometry_msgs::Point> footprint_;
    boost::shared_ptr<tf::TransformListener> tf_listener_;
    geometry_msgs::Point map_center_map;
    ros::Publisher pubTerrainModel;
    ros::Publisher pubexpandedStates;
    ros::Publisher pubPathRatingStates;
    ros::Publisher pubRobotMarker;
    pcl::PointCloud<pcl::PointXYZI> expandedStatesCloud;
    visualization_msgs::MarkerArray markers;
    pcl::PointXYZ current_robot_pose;
    float current_robot_orientation;

    int markerID;
    float flat_position_rating;

};
#endif
