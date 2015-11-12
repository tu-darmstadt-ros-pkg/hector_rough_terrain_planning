#include <hector_ground_contact_estimator/terrain_classifier.h>

namespace hector_ground_contact_estimator
{

bool add(TestModelService::Request &req,
         TestModelService::Response &res)
{
    res.a_srvc_out = req.a_srvc_in + req.testmodel_in.A;
    res.testmodel_out.A = 2*(req.a_srvc_in + req.testmodel_in.A);
    ROS_INFO("request received: x=%ld, y=%ld", (long int)req.a_srvc_in, (long int)req.testmodel_in.A);
    return true;
}


TerrainClassifier::TerrainClassifier()
    : frame_id("/world")
    , lock_input_cloud(false)
{
    setDataOutdated();
}


TerrainClassifier::TerrainClassifier(const TerrainClassifierParams &params)
    : frame_id("/world")
    , lock_input_cloud(false)
{
    setParams(params);
}

TerrainClassifier::~TerrainClassifier()
{
}

void TerrainClassifier::setParams(const TerrainClassifierParams &params)
{
    this->params = params;

    if (params.filter_mask & FILTER_PASS_THROUGH)           // cut out area of interest
        ROS_INFO("Using FILTER_PASS_THROUGH");
    if (params.filter_mask & FILTER_STATISTICAL_OUTLIER)    // remove outliers
        ROS_INFO("Using FILTER_STATISTICAL_OUTLIER");
    if (params.filter_mask & FILTER_VOXEL_GRID)             // summarize data
        ROS_INFO("Using FILTER_VOXEL_GRID");
    if (params.filter_mask & FILTER_MLS_SMOOTH)             // smooth data
        ROS_INFO("Using FILTER_MLS_SMOOTH");

    // clear old data
    if (cloud_input)
        cloud_processed.reset(new pcl::PointCloud<pcl::PointXYZ>(*cloud_input));
    setDataOutdated();
}

void TerrainClassifier::addCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, const std::string &frame_id)
{
    if (lock_input_cloud)
        return;

    this->cloud_input = cloud;
    this->frame_id = frame_id;

    // clear old data
    cloud_processed.reset(new pcl::PointCloud<pcl::PointXYZ>(*cloud_input));
    setDataOutdated();
}

void TerrainClassifier::showHeight(pcl::visualization::PCLVisualizer &viewer, const std::string &name, int viewport) const
{
    if (!cloud_height || cloud_height->empty())
    {
        ROS_WARN("showEdges was called before edges were detected!");
        return;
    }

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud_height, "intensity");
    viewer.addPointCloud<pcl::PointXYZI>(cloud_height, intensity_distribution, name + std::string("_cloud"), viewport);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name + std::string("_edges"), viewport);


}

const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &TerrainClassifier::getCloudInput() const
{
    if (!cloud_input || cloud_input->empty())
        ROS_WARN("getCloudInput was called before a input cloud was given!");
    return cloud_input;
}

const pcl::PointCloud<pcl::PointXYZ>::Ptr &TerrainClassifier::getCloudProcessed() const
{
    if (!cloud_processed || cloud_processed->empty())
        ROS_WARN("getCloudProcessed was called before a input cloud was given!");
    return cloud_processed;
}

const pcl::PointCloud<pcl::PointXYZ>::Ptr &TerrainClassifier::getCloudOutfiltered() const
{
    if (!cloud_outfiltered)
        ROS_WARN("getCloudOutfiltered was called before a input cloud was given!");
    return cloud_outfiltered;
}


void TerrainClassifier::getCloudProcessedLowRes(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const
{
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

    if (!cloud_points_with_normals || cloud_points_with_normals->empty())
        ROS_WARN("getCloudProcessedLow was called before normals were computed!");
    else
    {
        double half_low_res = 0.5*params.low_res;
        cloud->resize(cloud_points_with_normals->size());
        for (size_t i = 0; i < cloud->size(); i++)
        {
            cloud->at(i).x = cloud_points_with_normals->at(i).x;
            cloud->at(i).y = cloud_points_with_normals->at(i).y;
            cloud->at(i).z = cloud_points_with_normals->at(i).z - half_low_res; // reduce height so it matches vis with boxes
        }

        filterVoxelGrid<pcl::PointXYZ>(cloud,0.01,0.01,0.01);// params.low_res, params.low_res, 0.1);
    }
}



bool TerrainClassifier::computeNormals()
{
    // normals are up-to-date -> do nothing
    if (!cloud_normals_outdated && cloud_points_with_normals)
        return true;

    if (!cloud_input || cloud_input->empty())
    {
        ROS_ERROR("computeNormals was called but no point cloud was available");
        return false;
    }

    lock_input_cloud = true;

    // init map with standard data
    double ground_z = 0.0;
    //if (lower_body_state)
    //  ground_z = std::min(lower_body_state->left_foot_pose.position.z, lower_body_state->right_foot_pose.position.z);
    //else
    ROS_WARN("computeNormals: No state estimation of feet available. Assuming default ground level to be zero!");

    // preprocessing of cloud point
    cloud_processed.reset(new pcl::PointCloud<pcl::PointXYZ>(*cloud_input));
    if (params.filter_mask & FILTER_PASS_THROUGH)           // cut out area of interest
        filterPassThrough<pcl::PointXYZ>(cloud_processed, params.pt_field_name, ground_z + params.pt_min-0.3, ground_z + params.pt_max+0.3);
    if (params.filter_mask & FILTER_VOXEL_GRID)             // summarize data
        filterVoxelGrid<pcl::PointXYZ>(cloud_processed,0.03,0.03,0.03);// params.vg_lx, params.vg_ly, params.vg_lz);
    if (params.filter_mask & FILTER_MLS_SMOOTH)             // smooth data
        filterMlsSmooth<pcl::PointXYZ>(cloud_processed, params.ms_radius);

    if (cloud_processed->empty())
    {
        ROS_ERROR("Can compute normals. Got empty cloud point after filtering.");
        lock_input_cloud = false;
        return false;
    }

    // init normals data structure
    pcl::PointCloud<pcl::Normal> cloud_normals;
    cloud_points_with_normals.reset(new pcl::PointCloud<pcl::PointNormal>());

    // compute normals
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne(params.threads);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setInputCloud(cloud_processed);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(params.ne_radius);
    //ne.setKSearch(20);
    ne.compute(cloud_normals);

    pcl::concatenateFields(*cloud_processed, cloud_normals, *cloud_points_with_normals);

    // postprocessing
    if (params.filter_mask & FILTER_STATISTICAL_OUTLIER)    // remove outliers
    {
        cloud_outfiltered.reset(new pcl::PointCloud<pcl::PointXYZ>(*cloud_processed));
        filterStatisticalOutlier<pcl::PointNormal>(cloud_points_with_normals, params.so_k, params.so_radius);
        filterStatisticalOutlier<pcl::PointXYZ>(cloud_outfiltered, params.so_k, params.so_radius, true);
    }
    else
        cloud_outfiltered.reset(new pcl::PointCloud<pcl::PointXYZ>());

    // filter and some further postprocessing
    std::vector<int> indices;
    for (size_t i = 0; i < cloud_points_with_normals->size(); i++)
    {
        pcl::PointNormal &n = cloud_points_with_normals->at(i);
        if (!pcl_isfinite(n.normal_z))
            continue;

        indices.push_back((int)i);

        // flip all other normals in one direction
        if (n.normal_z < 0.0)
        {
            n.normal_x = -n.normal_x;
            n.normal_y = -n.normal_y;
            n.normal_z = -n.normal_z;
        }
    }

    // apply filtering
    pcl::PointCloud<pcl::PointNormal>::Ptr filtered(new pcl::PointCloud<pcl::PointNormal>());
    copyPointCloud(*cloud_points_with_normals, indices, *filtered);
    cloud_points_with_normals = filtered;

    cloud_normals_outdated = false;
    lock_input_cloud = false;
    return true;
}

bool TerrainClassifier::computeHeight()
{


    if (!computeNormals())
    {
        ROS_ERROR("Can't run edge detection!");
        return false;
    }

    lock_input_cloud = true;

    // init edge data structure
    cloud_height.reset(new pcl::PointCloud<pcl::PointXYZI>());
    cloud_height->resize(cloud_processed->size());

    // project all data to plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>());
    points->resize(cloud_processed->size());
    for (unsigned int i = 0; i < cloud_processed->size(); i++)
    {
        const pcl::PointXYZ &n = cloud_processed->at(i);
        pcl::PointXYZ &p = points->at(i);
        p.x = n.x;
        p.y = n.y;
        p.z = n.z;
    }


    for (size_t i = 0; i < points->size(); i++)
    {
        const pcl::PointXYZ &current = cloud_processed->at(i);
        pcl::PointXYZI &result = cloud_height->at(i);

        result.x = current.x;
        result.y = current.y;
        result.z = current.z;
        result.intensity = 0.0;

        if (result.z>0.5)
            result.intensity=0.5;
        else if (result.z<-0.1)
            result.intensity=-0.1;
        else
            result.intensity = result.z;




    }

    lock_input_cloud = false;
    return true;
}


void TerrainClassifier::setDataOutdated()
{
    cloud_normals_outdated = true;
    cloud_gradients_outdated = true;
}

void TerrainClassifier::getGridMapIndex(const nav_msgs::OccupancyGrid::ConstPtr &map, float x, float y, int &idx) const
{
    int map_x = floor((x-map->info.origin.position.x) * 1/map->info.resolution);
    int map_y = floor((y-map->info.origin.position.y) * 1/map->info.resolution);

    if (map_x < 0 || map->info.width <= (unsigned int)map_x || map_y < 0 || map->info.height <= (unsigned int)map_y)
        idx = -1;
    else
        idx = map_x + map_y * map->info.width;
}

void TerrainClassifier::getGridMapCoords(const nav_msgs::OccupancyGrid::ConstPtr &map, unsigned int idx, float &x, float &y) const
{
    unsigned int map_x = idx % map->info.width;
    unsigned int map_y = std::floor(idx / map->info.width);

    x = (float)map_x * map->info.resolution + map->info.origin.position.x;
    y = (float)map_y * map->info.resolution + map->info.origin.position.y;
}

}
