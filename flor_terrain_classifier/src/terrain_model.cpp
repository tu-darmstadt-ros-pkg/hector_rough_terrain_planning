#include <flor_terrain_classifier/terrain_model.h>

namespace hector_terrain_model
{

TerrainModel::TerrainModel()
{
    viewer_init = false;
}

TerrainModel::TerrainModel(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    viewer_init = false;
    cloud_processed=cloud;
    cloud_processed_Ptr= pcl::PointCloud<pcl::PointXYZ>::Ptr (&cloud_processed);
}

TerrainModel::~TerrainModel()
{

}

void TerrainModel::updateCloud(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    cloud_processed=cloud;
    cloud_processed_Ptr= pcl::PointCloud<pcl::PointXYZ>::Ptr (&cloud_processed);
}


void TerrainModel::updateViewer(pcl::visualization::PCLVisualizer update_viewer){
    viewer_init = false;
    viewer = update_viewer;
}


/*--------------------------------------------------------------------------------------------*/
/* ---------------Helper Functions for Eigen Vector3 and PointXYZ ----------------------------*/
/*--------------------------------------------------------------------------------------------*/

// TODO unused or use it
bool pointsEqual(pcl::PointXYZ p1, pcl::PointXYZ p2){
    /*float delta = 0.0000001;
    if ((fabs(p1.x - p2.x)) < delta &&
            (fabs(p1.y - p2.y)) < delta &&
            (fabs(p1.z - p2.z)) < delta)
        return true;*/
    if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z)
        return true;
    else
        return false;
}
// normal adding, subtracting points
pcl::PointXYZ addPointVector(const pcl::PointXYZ& p1,const  pcl::PointXYZ& p2){
    return pcl::PointXYZ(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
}

pcl::PointXYZ addPointVector(const pcl::PointXYZ& p1,const  Eigen::Vector3f& vector){
    return pcl::PointXYZ(p1.x + vector.x(), p1.y + vector.y(), p1.z + vector.z());
}

pcl::PointXYZ subtractPoints(const pcl::PointXYZ& p1,const pcl::PointXYZ& p2){
    return pcl::PointXYZ(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

Eigen::Vector3f subtractPointsEigen(const pcl::PointXYZ& p1,const pcl::PointXYZ& p2){
    return Eigen::Vector3f(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

// z stays
pcl::PointXYZ rotatePoint(pcl::PointXYZ p, float degree /*radiants*/){
    return pcl::PointXYZ(cos(degree)*p.x - sin(degree)*p.y,
                         sin(degree)*p.x + cos(degree)*p.y,
                         p.z);
}

float dotproductEigen(Eigen::Vector3f v1, Eigen::Vector3f v2){
    return (v1.x() * v2.x() +
            v1.y() * v2.y() +
            v1.z() * v2.z());
}

float dotProduct(const pcl::PointXYZ& a,const  pcl::PointXYZ& b){
    return a.x*b.x+a.y*b.y+a.z*b.z;
}

pcl::PointXYZ crossProduct(const pcl::PointXYZ& a,const  pcl::PointXYZ& b){
    float cx=a.y*b.z-a.z*b.y;
    float cy=a.z*b.x-a.x*b.z;
    float cz=a.x*b.y-a.y*b.x;
    return pcl::PointXYZ(cx,cy,cz);
}

// returns angle value in degree
float angleBetween(Eigen::Vector3f v1, Eigen::Vector3f v2){
    float PI = 3.14159265;
    float acosvalue = dotproductEigen(v1,v2)/
            (sqrt(dotproductEigen(v1,v1)) * sqrt(dotproductEigen(v2,v2)));
    if (acosvalue > 0.9999999){
        return 0.0;
    }
    if (acosvalue < -0.999999){
        return 180.0;
    }
    float result = acos(acosvalue) * (180.0 / PI);

    return result;
}

// distance between 2 points (only seen in Z = 0 plane)
float distanceXY(const pcl::PointXYZ p1, const pcl::PointXYZ p2){
    return sqrt((p1.x-p2.x)*(p1.x-p2.x)+
                (p1.y-p2.y)*(p1.y-p2.y));
}

// distance between 2 points - 3D
float distanceXYZ(const pcl::PointXYZ p1, const pcl::PointXYZ p2){
    return sqrt((p1.x-p2.x)*(p1.x-p2.x)+
                (p1.y-p2.y)*(p1.y-p2.y)+
                (p1.z-p2.z)*(p1.z-p2.z));
}

// computes distance between straight / axis (ps, vector) and the point p
float distancePointStraight(pcl::PointXYZ ps, pcl::PointXYZ vector, pcl::PointXYZ p){
    pcl::PointXYZ perpendicular = crossProduct((subtractPoints(p,ps)), vector);
    float d = sqrt(dotProduct(perpendicular, perpendicular)) / sqrt(dotProduct(vector, vector));
    return d;
}

// 1 if positive, 0 if zero, -1 if negative
int sign(float a){
    if (a == 0) return 0;
    if (a > 0) return 1;
    return -1;
}

// checks if 2 vectors are similar; needed for deletion of convex_hull_ dge. TODO check reference / copy
/*bool check_similar_vector(pcl::PointXYZ support_point_1, pcl::PointXYZ support_point_2,
                          pcl::PointXYZ hull_p1, pcl::PointXYZ  hull_p2,
                          float similarity_distance, float similarity_angle){
    // check angle similarity
    float angle1 = angleBetween(subtractPointsEigen(support_point_1, support_point_2), subtractPointsEigen(hull_p2, hull_p1));
    float angle2 = angleBetween(subtractPointsEigen(support_point_1, support_point_2), subtractPointsEigen(hull_p1, hull_p2));
    float angle = fmin(angle1, angle2);
    if (angle > similarity_angle)
        return false;
    //check distance similarity evaluated by point distances
    float distance1 = (distanceXYZ(support_point_1, hull_p1) + distanceXYZ(support_point_2, hull_p2)) / 2.0;
    float distance2 = (distanceXYZ(support_point_1, hull_p2) + distanceXYZ(support_point_2, hull_p1)) / 2.0;
    float average_dist = fmin(distance1, distance2);

    if (average_dist > similarity_distance)
        return false;
    return true;
}*/

// returns a value how similar vectors are to each other. close to 0 is very similar, 0.50 would not be similar anymore.
float vectorSimilarity(pcl::PointXYZ support_point_1, pcl::PointXYZ support_point_2,
                       pcl::PointXYZ hull_p1, pcl::PointXYZ  hull_p2){
    // check angle similarity
    float angle1 = angleBetween(subtractPointsEigen(support_point_1, support_point_2), subtractPointsEigen(hull_p2, hull_p1));
    float angle2 = angleBetween(subtractPointsEigen(support_point_1, support_point_2), subtractPointsEigen(hull_p1, hull_p2));
    float angle = fmin(angle1, angle2);
    // value is 1 for 1 degree

    //check distance similarity evaluated by point distances
    float distance1 = (distanceXYZ(support_point_1, hull_p1) + distanceXYZ(support_point_2, hull_p2));
    float distance2 = (distanceXYZ(support_point_1, hull_p2) + distanceXYZ(support_point_2, hull_p1));
    float distance = fmin(distance1, distance2);
    // value is 0.01 for 1 centimeters

    // 1 degree is same value as 2 centimeter offset
    // example 10 centimeters offset (eg 2 times 5cm) and 7 degree -> value will be 0.20 + 0.07 = 0.27
    return distance*20.0 + angle/100.0;
}

// return > 0 -> counterclockwise, // return < 0 -> clockwise  // return = 0 -> neither nor
float ccw(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3){
    return (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
}


/*--------------------------------------------------------------------------------------------*/
/* --------------Helper Functoins end---------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------*/


float TerrainModel::planeDistance(const pcl::PointXYZ& testpoint, const pcl::PointXYZ& plane_n, const pcl::PointXYZ& plane_p)
{
    float d=dotProduct(pcl::PointXYZ(testpoint.x-plane_p.x, testpoint.y-plane_p.y, testpoint.z-plane_p.z), plane_n) /
            sqrt(plane_n.x*plane_n.x+plane_n.y*plane_n.y+plane_n.z*plane_n.z);
    return d;
}

pcl::PointXYZ TerrainModel::planeProjection(const pcl::PointXYZ& projection_p, const pcl::PointXYZ& plane_n, const pcl::PointXYZ& plane_p)
{


    Eigen::Vector3f x = Eigen::Vector3f(projection_p.x,projection_p.y,projection_p.z);
    Eigen::Vector3f n = Eigen::Vector3f(plane_n.x,plane_n.y,plane_n.z);
    Eigen::Vector3f r = Eigen::Vector3f(plane_p.x,plane_p.y,plane_p.z);
    Eigen::Vector3f res = x - ((x-r).dot(n)/n.dot(n))*n;
    pcl::PointXYZ ret = pcl::PointXYZ(res[0],res[1],res[2]);
    return ret;
}



bool TerrainModel::atPlaneTest(const pcl::PointXYZ& testpoint, const pcl::PointXYZ& plane_n, const pcl::PointXYZ& plane_p, const float& delta)
{
    return (abs(planeDistance(testpoint,plane_n,plane_p))<=delta);
}




//convex hull computation
//  http://en.wikipedia.org/wiki/Gift_wrapping_algorithm
// first point is also last point
void convex_hull_comp(pcl::PointCloud<pcl::PointXYZ>& cloud,std::vector<unsigned int>& convex_hull_indices)
{
    float x_min=cloud.at(0).x;
    unsigned int point_on_hull=0;
    pcl::PointCloud<pcl::PointXYZ> cloud_2d;
    cloud_2d.resize(0);

    //find minx
    for (unsigned int i = 0; i < cloud.size(); i++)
    {
        pcl::PointXYZ p = cloud.at(i);
        cloud_2d.push_back(pcl::PointXYZ(p.x,p.y,0.0));
        if (p.x<x_min)
        {
            x_min=p.x;
            point_on_hull=i;
        }

    }
    //build hull
    int i=0;
    unsigned int endpoint;
    while (true)
    {
        convex_hull_indices.push_back(point_on_hull);
        endpoint=0;
        if((0==i)&&(0==point_on_hull))endpoint=1;
        for(unsigned int j=1; j<cloud_2d.size();++j)
        {
            pcl::PointXYZ& p0 =cloud_2d.at(convex_hull_indices.at(i));
            pcl::PointXYZ& p1 =cloud_2d.at(endpoint);
            pcl::PointXYZ& p2 =cloud_2d.at(j);
            float ccw_f=ccw(cloud_2d.at(convex_hull_indices.at(i)),cloud_2d.at(endpoint),cloud_2d.at(j));
            float dist_old=(p0.x-p1.x)*(p0.x-p1.x)+(p0.y-p1.y)*(p0.y-p1.y);
            float dist_new=(p0.x-p2.x)*(p0.x-p2.x)+(p0.y-p2.y)*(p0.y-p2.y);
            bool isleft=(ccw_f <0);
            bool isfurther=(ccw_f<=0.00000 && dist_new>dist_old);
            if(endpoint==point_on_hull || isleft || isfurther)
            {
                endpoint=j;
            }
        }

        i++;
        point_on_hull=endpoint;
        if(endpoint==convex_hull_indices.at(0))
        {
            // first point is also last point
            convex_hull_indices.push_back(convex_hull_indices.at(0));
            break;
        }

    }
}

//As proposed in "Modeling the manipulator and flipper pose effects on tip over stability of a tracked mobile manipulator" by Chioniso Dube
// a low number is not stable. a high number is stable.
std::vector<float> computeForceAngleStabilityMetric(const pcl::PointXYZ& center_of_mass_pcl, std::vector<pcl::PointXYZ>& convex_hull_points_pcl)
{
    const Eigen::Vector3f center_of_mass = Eigen::Vector3f(center_of_mass_pcl.x,center_of_mass_pcl.y,center_of_mass_pcl.z);
    std::vector<Eigen::Vector3f> p;
    Eigen::Vector3f f_r= Eigen::Vector3f(0.0,0.0,-10.0);

    for(unsigned int i=0; i<convex_hull_points_pcl.size();++i)
    {
        pcl::PointXYZ& pi = convex_hull_points_pcl.at(i);
        p.push_back(Eigen::Vector3f(pi.x,pi.y,pi.z));
    }
    std::vector<float> res;
    for(unsigned int i=0; i<(p.size()-1);++i)
    {
        Eigen::Vector3f p_i1;
        if(p.size()==1) p_i1 = p.at(0);
        else p_i1 = p.at(i+1);
        Eigen::Vector3f ai=p_i1-p.at(i);
        ai.normalize();
        Eigen::Matrix3f ident_mat =Eigen::Matrix3f::Identity();
        Eigen::Vector3f li=(ident_mat-ai*ai.transpose())*(p_i1-center_of_mass);
        Eigen::Vector3f fi=(ident_mat-ai*ai.transpose())*f_r;
        Eigen::Vector3f li_norm=li;
        Eigen::Vector3f fi_norm=fi;
        li_norm.normalize();
        fi_norm.normalize();
        Eigen::Vector3f di= (-li)+(li.dot(fi_norm))*fi_norm;
        float theta_i=acos(li_norm.dot(fi_norm));
        float sigma_i=((fi_norm.cross(li_norm)).dot(ai))?1.0:-1.0 ;


        float beta=sigma_i*theta_i*di.norm()*f_r.norm();
        res.push_back(beta);
    }

    return res;
}



// evaluates the second or third point of the supporting plane
// if false is returned, there could not be a point found.
bool TerrainModel::findSupportPoint(const pcl::PointXYZ& tip_over_axis_point,
                                         const pcl::PointXYZ& tip_over_axis_vector,
                                         const pcl::PointCloud<pcl::PointXYZI>& pointcloud_robo,
                                         const pcl::PointXYZ& tip_over_direction,
                                         pcl::PointXYZ& support_point)
{

    // PARAMETER
    float minimum_distance = 0.15; // minimum distance in which a support point can be found from the last one.

    pcl::PointCloud<pcl::PointXYZ> cloud_projected;
    cloud_projected.resize(pointcloud_robo.size());
    for(unsigned int i=0; i<cloud_projected.size();++i)
    {
        pcl::PointXYZ &p_pro= cloud_projected.at(i);
        const pcl::PointXYZI &p_pos= pointcloud_robo.at(i);
        p_pro.x=p_pos.x;
        p_pro.y=p_pos.y;
        p_pro.z=p_pos.z;
        const  pcl::PointXYZ &p=p_pro;

        cloud_projected.at(i)=planeProjection(p,tip_over_axis_vector,tip_over_axis_point);
    }


    //find supppoint
    float min_angle=360.0;

    int min_angle_idx = -1;
    const pcl::PointXYZ v1 = tip_over_direction;


    //states the direction of the points if on one side of the tip_over_axis, or on the other
    // 1 = counterclockwise, -1 = clockwise
    const int directionccw = sign(ccw (tip_over_axis_point,
                                       addPointVector(tip_over_axis_point, tip_over_axis_vector),
                                       addPointVector(tip_over_axis_point, tip_over_direction)));


    for(unsigned int i=0; i<cloud_projected.size();++i)
    {
        pcl::PointXYZ &p_pro= cloud_projected.at(i);
        const pcl::PointXYZ v2 = pcl::PointXYZ(p_pro.x-tip_over_axis_point.x,p_pro.y-tip_over_axis_point.y,p_pro.z-tip_over_axis_point.z);
        float angle= acos( dotProduct(v1,v2)/sqrt(dotProduct(v1,v1)*dotProduct(v2,v2))); // ERROR???? sqrt(dotproduct) * sqrt(dotproduct)

        const float pi = 3.14159;
        angle = angle * 360.0/(2.0*pi);

        if (angle>90.0) angle=180.0-angle;
        // which side to the axis
        pcl::PointXYZ ps=pcl::PointXYZ( pointcloud_robo.at(i).x,pointcloud_robo.at(i).y,pointcloud_robo.at(i).z);
        const int side = sign(ccw(tip_over_axis_point,
                                  addPointVector(tip_over_axis_point, tip_over_axis_vector),
                                  ps));

        if (angle<min_angle &&
                side == directionccw)
        {
            pcl::PointXYZ current = pcl::PointXYZ(pointcloud_robo.at(i).x,pointcloud_robo.at(i).y,pointcloud_robo.at(i).z);
            if (distancePointStraight(tip_over_axis_point, tip_over_axis_vector, current) > minimum_distance){
                min_angle=angle;
                min_angle_idx=i;
            }
        }

    }

    // IF NO POINT COULD BE FOUND
    if (min_angle_idx == -1){
        return false;
    }

    support_point=pcl::PointXYZ(pointcloud_robo.at(min_angle_idx).x,pointcloud_robo.at(min_angle_idx).y,pointcloud_robo.at(min_angle_idx).z);
    return true;
}



/* builds the convex hull with the given supportpoints */
std::vector<pcl::PointXYZ> TerrainModel:: buildConvexHull(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_positionRating,
                                                               const pcl::PointXYZ& check_pos,
                                                               const pcl::PointXYZ& support_point_1,
                                                               const pcl::PointXYZ& support_point_2,
                                                               const pcl::PointXYZ& support_point_3,
                                                               const bool iterative, // if this is true, points on one side of supp1, supp2 will not be considerred
                                                               std::vector<unsigned int>& convex_hull_indices, // empty before call
                                                               pcl::PointCloud<pcl::PointXYZ>::Ptr ground_contact_points){ // empty before call

    float delta_for_contact = 0.015; //  +- in m
    // PARAMETER smoothing
    bool distance_smoothing = true;
    bool angle_smoothing = true;
    float smooth_max_angle = 20.0;
    float smooth_max_distance = 0.05;


    const pcl::PointXYZ final_normal= crossProduct(pcl::PointXYZ(support_point_1.x-support_point_2.x,support_point_1.y-support_point_2.y,support_point_1.z-support_point_2.z),
                                                   pcl::PointXYZ(support_point_1.x-support_point_3.x,support_point_1.y-support_point_3.y,support_point_1.z-support_point_3.z));


    //Find ground contact points
    for (unsigned int i = 0; i < cloud_positionRating->size(); i++)
    {
        pcl::PointXYZI& p = cloud_positionRating->at(i);
        const float dist = planeDistance(pcl::PointXYZ(p.x,p.y,p.z),final_normal,support_point_1);
        if(fabs(dist) < delta_for_contact)
        {
            if (!iterative){
                ground_contact_points->push_back(pcl::PointXYZ(p.x,p.y,p.z));
            }
            else {
                // TODO diesen Punkt umwandeln ist nicht perfomant // sollte aber nicht bei so vielen der fall sein.
                pcl::PointXYZ pointXYZ = pcl::PointXYZ(p.x, p.y, p.z);
                // only add points on the right side of the supp points 1 and 2
                if (sign(ccw(support_point_1, support_point_2, support_point_3)) == sign(ccw(support_point_1, support_point_2, pointXYZ))
                        || ccw(support_point_1, support_point_2, pointXYZ) == 0){
                    ground_contact_points->push_back(pcl::PointXYZ(p.x,p.y,p.z));
                }
            }
        }
        p.intensity=p.z;
    }


    //const pcl::PointXYZ pcs=planeProjection(check_pos,final_normal,support_point_1);
    //viewer.addSphere(pcs,0.04,1,0,1, "cp_pro", viewport);  ???

    // convex hull build here
    // first point is also last point
    convex_hull_comp(*ground_contact_points, convex_hull_indices);

    std::vector<pcl::PointXYZ> convex_hull_points;

    // add points to the list
    for(int i=0; i<(convex_hull_indices.size()); ++i)
    {
        const pcl::PointXYZ p1(ground_contact_points->at(convex_hull_indices[i]).x,ground_contact_points->at(convex_hull_indices[i]).y,ground_contact_points->at(convex_hull_indices[i]).z);
        convex_hull_points.push_back(p1);
    }

    if (distance_smoothing){
        ROS_INFO("distance smoothing");
        // distance smoothing
        for (unsigned int i = 0; i < convex_hull_points.size() - 1; i = i){
            if (i < convex_hull_points.size()-2){
                if (distanceXYZ(convex_hull_points.at(i), convex_hull_points.at(i+1)) < smooth_max_distance){
                    // keep the point which is further away from the others (try not make area smaller)
                    unsigned int offset = 0;
                    float dist_p_i = 0.0;
                    float dist_p_next = 0.0;
                    for (unsigned int k = 0; k < convex_hull_points.size() - 1; k++){
                        dist_p_i = dist_p_i + pow(distanceXYZ(convex_hull_points.at(i), convex_hull_points.at(k)), 2);
                        dist_p_next = dist_p_next + pow(distanceXYZ(convex_hull_points.at(i+1), convex_hull_points.at(k)),2);
                    }
                    /*if (distanceXYZ(convex_hull_points.at(i), check_pos) > distanceXYZ(convex_hull_points.at(i+1), check_pos)){
                    offset = 1;
                }*/
                    if (dist_p_i > dist_p_next){
                        offset = 1;
                    }
                    convex_hull_points.erase(convex_hull_points.begin()+(i+offset));
                    convex_hull_indices.erase(convex_hull_indices.begin()+(i+offset));
                }
                else {
                    i++;
                }
            }
            // same thing for last (= first) point
            else {
                if (distanceXYZ(convex_hull_points.at(i), convex_hull_points.at(i+1)) < smooth_max_distance){
                    // a point must be deleted
                    float dist_p_i = 0.0;
                    float dist_p_next = 0.0;
                    for (unsigned int k = 0; k < convex_hull_points.size() - 1; k++){
                        dist_p_i = dist_p_i + pow(distanceXYZ(convex_hull_points.at(i), convex_hull_points.at(k)),2);
                        dist_p_next = dist_p_next + pow(distanceXYZ(convex_hull_points.at(i+1), convex_hull_points.at(k)),2);
                    }
                    if (dist_p_i < dist_p_next){
                        /*if (distanceXYZ(convex_hull_points.at(i), check_pos) < distanceXYZ(convex_hull_points.at(i+1), check_pos)){*/
                        // second last must be removed
                        convex_hull_points.erase(convex_hull_points.begin()+(i));
                        convex_hull_indices.erase(convex_hull_indices.begin()+(i));
                    }
                    else{
                        // last = first must be removed
                        convex_hull_points.erase(convex_hull_points.begin()+(i+1));
                        convex_hull_points.at(0) = convex_hull_points.at(i);
                        convex_hull_indices.erase(convex_hull_indices.begin()+(i+1));
                        convex_hull_indices.at(0) = convex_hull_indices.at(i);
                    }
                }
                break; // needed because loop is doing i = i // could also be i++
            }
        }

        ROS_INFO("end distance smoothing");
    }

        // angle smoothing
        if (angle_smoothing){

            ROS_INFO("angle smoothing");
            for (unsigned int i = 0; i < convex_hull_points.size() - 1; i = i){
                if (i < convex_hull_points.size()-2){

                    Eigen::Vector3f direction1 = subtractPointsEigen(convex_hull_points.at(i + 1),convex_hull_points.at(i));
                    Eigen::Vector3f direction2 = subtractPointsEigen(convex_hull_points.at(i + 2),convex_hull_points.at(i+1));
                    float angle = angleBetween(direction1, direction2);

                    // delete the point if angle is very low
                    if (angle < smooth_max_angle){
                        convex_hull_points.erase(convex_hull_points.begin()+(i+1));
                        convex_hull_indices.erase(convex_hull_indices.begin()+(i+1));
                    }

                    else {
                        ++i;
                    }
                }
                // same thing for last (= first) point
                else {
                    Eigen::Vector3f directionlast = subtractPointsEigen(convex_hull_points.at(i + 1),convex_hull_points.at(i));
                    Eigen::Vector3f directionfirst = subtractPointsEigen(convex_hull_points.at(1),convex_hull_points.at(0));
                    float angle = angleBetween(directionlast, directionfirst);

                    if (angle < smooth_max_angle){
                        //delete last ( = first) point
                        convex_hull_points.erase(convex_hull_points.begin()+(i+1));
                        convex_hull_points.at(0) = convex_hull_points.at(i);
                        convex_hull_indices.erase(convex_hull_indices.begin()+(i+1));
                        convex_hull_indices.at(0) = convex_hull_indices.at(i);
                    }
                    break; // needed because loop is doing i = i // could also be i++
                }
            }
            ROS_INFO("end angle smoothing");
        }



    return convex_hull_points;
}

void TerrainModel::computeRobotPosition(const pcl::PointXYZ support_point_1, const pcl::PointXYZ support_point_2, const pcl::PointXYZ support_point_3,
                                             const pcl::PointXYZ p0, const pcl::PointXYZ p1, const pcl::PointXYZ p2, const pcl::PointXYZ p3,
                                             const Eigen::Vector3f& offset_CM,
                                             pcl::PointXYZ& normal,
                                             pcl::PointXYZ& robot_point_0, pcl::PointXYZ& robot_point_1, pcl::PointXYZ& robot_point_2, pcl::PointXYZ& robot_point_3,
                                             pcl::PointXYZ& robot_point_mid, pcl::PointXYZ& robot_center_of_mass){

    normal= crossProduct(pcl::PointXYZ(support_point_1.x-support_point_2.x,support_point_1.y-support_point_2.y,support_point_1.z-support_point_2.z),
                         pcl::PointXYZ(support_point_1.x-support_point_3.x,support_point_1.y-support_point_3.y,support_point_1.z-support_point_3.z));
    if (normal.z < 0){
        normal.x = -normal.x;
        normal.y = -normal.y;
        normal.z = -normal.z;
    }

    /* ALT float z0=(-(x_max-support_point_2.x)*normal.x-(y_max-support_point_2.y)*normal.y)/normal.z +support_point_2.z;
    float z1=(-(x_min-support_point_2.x)*normal.x-(y_max-support_point_2.y)*normal.y)/normal.z +support_point_2.z;
    float z2=(-(x_max-support_point_2.x)*normal.x-(y_min-support_point_2.y)*normal.y)/normal.z +support_point_2.z;
    float z3=(-(x_min-support_point_2.x)*normal.x-(y_min-support_point_2.y)*normal.y)/normal.z +support_point_2.z;
*/
    float z0=(-(p0.x-support_point_2.x)*normal.x-(p0.y-support_point_2.y)*normal.y)/normal.z +support_point_2.z;
    float z1=(-(p1.x-support_point_2.x)*normal.x-(p1.y-support_point_2.y)*normal.y)/normal.z +support_point_2.z;
    float z2=(-(p2.x-support_point_2.x)*normal.x-(p2.y-support_point_2.y)*normal.y)/normal.z +support_point_2.z;
    float z3=(-(p3.x-support_point_2.x)*normal.x-(p3.y-support_point_2.y)*normal.y)/normal.z +support_point_2.z;

    robot_point_0 = pcl::PointXYZ(p0.x, p0.y,z0);
    robot_point_1 = pcl::PointXYZ(p1.x, p1.y, z1);
    robot_point_2 = pcl::PointXYZ(p2.x, p2.y, z2);
    robot_point_3 = pcl::PointXYZ(p3.x, p3.y, z3);

    robot_point_mid = addPointVector(robot_point_0, addPointVector(robot_point_1, addPointVector(robot_point_2, robot_point_3)));
    robot_point_mid.x = robot_point_mid.x / 4.0;
    robot_point_mid.y = robot_point_mid.y / 4.0;
    robot_point_mid.z = robot_point_mid.z / 4.0;



    // compute center_of_mass


    Eigen::Vector3f normal_vector = Eigen::Vector3f(normal.x, normal.y, normal.z);
    normal_vector.normalize();
    normal.x = normal_vector.x();
    normal.y = normal_vector.y();
    normal.z = normal_vector.z();

    // ERROR ? ACHTUNG EVENTUELL FLASCH RUM, DA ICH NICHT WEISS WIE RUM DER ROBOTER STEHT
    // --------------------------------------------------------------------------------------------
    // bezg p_pro 1 / 0
    robot_center_of_mass = computeCenterOfMass(robot_point_0, robot_point_1, normal_vector, robot_point_mid, offset_CM);



}

pcl::PointXYZ TerrainModel::computeCenterOfMass(const pcl::PointXYZ &p1_left,
                                                     const pcl::PointXYZ &p2_left,
                                                     const Eigen::Vector3f &normal,
                                                     const pcl::PointXYZ &mid,
                                                     const Eigen::Vector3f &offset){

    Eigen::Vector3f axis_left = Eigen::Vector3f(p2_left.x - p1_left.x, p2_left.y - p1_left.y, p2_left.z - p1_left.z);
    axis_left.normalize();
    Eigen::Vector3f my_normal = normal;
    my_normal.normalize();

    float CMx = mid.x + (offset.z() * my_normal.x()) + (axis_left.x() * offset.x());
    float CMy = mid.y + (offset.z() * my_normal.y()) + (axis_left.y() * offset.x());
    float CMz = mid.z + (offset.z() * my_normal.z()) + (axis_left.z() * offset.x());

    pcl::PointXYZ center_of_mass = pcl::PointXYZ(CMx, CMy, CMz);
    return center_of_mass;
}

// orientation in radiants
bool TerrainModel::computePositionRating(const pcl::PointXYZ& check_pos,
                                              const float orientation,
                                              float position_rating, // after tipping over
                                              //float contact_area, // of the first polygon // TODO
                                              int unstable_axis, // of the first polygon
                                              pcl::visualization::PCLVisualizer &viewer,
                                              int viewport)
{
    unstable_axis = 0;

    // PARAMETER tipp over robot scale
    bool tip_over_active = true;
    bool draw_convex_hull_first_polygon = true;
    bool draw_convex_hull_iterative = true;
    bool draw_convex_hull_iterative_ground_points= true;

    Eigen::Vector3f offset_CM = Eigen::Vector3f(0.0,0.0,0.15);

    float width=0.50; // y
    float length=0.70; // x

    // Points under robot
    cloud_positionRating.reset(new pcl::PointCloud<pcl::PointXYZI>());
    unsigned int highest_Point_idx;
    unsigned int n_counter=0;

    pcl::PointXYZ p0=pcl::PointXYZ(+ 0.5*length,
                                   + 0.5*width,
                                   0);
    p0 = rotatePoint(p0, orientation);
    p0 = addPointVector(p0, check_pos);

    pcl::PointXYZ p1=pcl::PointXYZ(- 0.5*length,
                                   + 0.5*width,
                                   0);
    p1 = rotatePoint(p1, orientation);
    p1 = addPointVector(p1, check_pos);

    pcl::PointXYZ p2=pcl::PointXYZ(- 0.5*length,
                                   - 0.5*width,
                                   0);
    p2 = rotatePoint(p2, orientation);
    p2 = addPointVector(p2, check_pos);

    pcl::PointXYZ p3=pcl::PointXYZ(+ 0.5*length,
                                   - 0.5*width,
                                   0);
    p3 = rotatePoint(p3, orientation);
    p3 = addPointVector(p3, check_pos);


    //corner points of robot with z = 0
    /*const pcl::PointXYZ p0=pcl::PointXYZ(check_pos.x + (cos(orientation)*length*0.5 - sin(orientation)*width*0.5),
                                    check_pos.y + (sin(orientation)*length*0.5 + cos(orientation)*width*0.5),
                                    0);
     const pcl::PointXYZ p1=pcl::PointXYZ(check_pos.x - (cos(orientation)*length*0.5 - sin(orientation)*width*0.5),
                                    check_pos.y + (sin(orientation)*length*0.5 + cos(orientation)*width*0.5),
                                    0);
     const pcl::PointXYZ p2=pcl::PointXYZ(check_pos.x - (cos(orientation)*length*0.5 - sin(orientation)*width*0.5),
                                    check_pos.y - (sin(orientation)*length*0.5 + cos(orientation)*width*0.5),
                                    0);
     const pcl::PointXYZ p3=pcl::PointXYZ(check_pos.x + (cos(orientation)*length*0.5 - sin(orientation)*width*0.5),
                                    check_pos.y - (sin(orientation)*length*0.5 + cos(orientation)*width*0.5),
                                    0);
*/
    /*  ROS_INFO("p0 after %f %f %f", p0.x, p0.y, p0.z);
     ROS_INFO("p1 after %f %f %f", p1.x, p1.y, p1.z);
     ROS_INFO("p2 after %f %f %f", p2.x, p2.y, p2.z);
     ROS_INFO("p3 after %f %f %f", p3.x, p3.y, p3.z); */  // kann weg


    bool hull_cpp= (ccw(p0,p1,p2)<0);
    for (unsigned int i = 0; i < cloud_processed_Ptr->size(); i++)
    {
        const  pcl::PointXYZ &pp= cloud_processed_Ptr->at(i);

        bool c0=hull_cpp ? (ccw(p0,p1,pp)<0) : (ccw(p0,p1,pp)>0);
        bool c1=hull_cpp ? (ccw(p1,p2,pp)<0) : (ccw(p1,p2,pp)>0);
        bool c2=hull_cpp ? (ccw(p2,p3,pp)<0) : (ccw(p2,p3,pp)>0);
        bool c3=hull_cpp ? (ccw(p3,p0,pp)<0) : (ccw(p3,p0,pp)>0);

        if(c0&&c1&&c2&&c3)
        {
            pcl::PointXYZI p= pcl::PointXYZI();
            p.x=pp.x;
            p.y=pp.y;
            p.z=pp.z;
            p.intensity=0.0;
            cloud_positionRating->push_back(p);

            if(n_counter==0) highest_Point_idx=1;
            else if(p.z>cloud_positionRating->at(highest_Point_idx).z) highest_Point_idx=n_counter;
            ++n_counter;
        }
    }

    if(cloud_positionRating->size()==0)
    {
        ROS_ERROR("[flor terrain classifier] cloud size is 0");
        return false;
    }


    pcl::PointXYZI &p_max= cloud_positionRating->at(highest_Point_idx);//highest point
    int support_point_1_idx;
    float min_dist=-1.0;
    //choose highest point (+-delta), closest to CoM
    for (unsigned int i = 0; i < cloud_positionRating->size(); i++)
    {
        pcl::PointXYZI& p = cloud_positionRating->at(i);
        if(((p.z-p_max.z)<0.03)&&((p.z-p_max.z)>-0.03))
        {
            float dist=sqrt((p.x-check_pos.x)*(p.x-check_pos.x)+(p.y-check_pos.y)*(p.y-check_pos.y));
            if(min_dist<0.0 || dist<min_dist)
            {
                min_dist=dist;
                support_point_1_idx=i;
                // ("i d : %i %f %f ",i,min_dist, dist);
            }

        }
    }
    pcl::PointXYZ support_point_1 = pcl::PointXYZ(cloud_positionRating->at(support_point_1_idx).x,cloud_positionRating->at(support_point_1_idx).y,cloud_positionRating->at(support_point_1_idx).z);


    // is used after while loop, but initiated in it.
    pcl::PointXYZ support_point_2;
    pcl::PointXYZ support_point_3;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_contact_points(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<unsigned int> convex_hull_indices;
    std::vector<pcl::PointXYZ> convex_hull_points;

    int counter = 0;
    while (true){


        // find supp P 2
        const pcl::PointXYZ tip_over_axis_point = support_point_1;
        const pcl::PointXYZ tip_over_axis_vector = (crossProduct(pcl::PointXYZ(support_point_1.x-check_pos.x,support_point_1.y-check_pos.y,0),pcl::PointXYZ(0,0,1)));
        const pcl::PointXYZ tip_over_direction = pcl::PointXYZ(check_pos.x - support_point_1.x, check_pos.y - support_point_1.y, 0);

        bool point_evaluated = findSupportPoint(tip_over_axis_point,
                                                tip_over_axis_vector,
                                                (*cloud_positionRating),
                                                tip_over_direction,
                                                support_point_2);
        if (!point_evaluated)
            ROS_ERROR("[flor_terrain_classifier::compute_position_rating] no support_point found ");

        //find third point
        const pcl::PointXYZ tip_over_axis_point_3 = support_point_1;
        const pcl::PointXYZ tip_over_axis_vector_3= subtractPoints(support_point_2, support_point_1);
        pcl::PointXYZ tip_over_direction_3 ;
        if(ccw(support_point_1,support_point_2,check_pos)>0)
            tip_over_direction_3 = crossProduct(tip_over_axis_vector_3, pcl::PointXYZ(0,0,-1));
        else
            tip_over_direction_3 = crossProduct(tip_over_axis_vector_3, pcl::PointXYZ(0,0,1));

        point_evaluated = findSupportPoint(tip_over_axis_point_3,
                                           tip_over_axis_vector_3,
                                           (*cloud_positionRating),
                                           tip_over_direction_3,
                                           support_point_3);
        if (!point_evaluated)
            ROS_ERROR("[flor_terrain_classifier::compute_position_rating] no support_point found ");


        convex_hull_points = buildConvexHull(cloud_positionRating,
                                             check_pos,
                                             support_point_1,
                                             support_point_2,
                                             support_point_3,
                                             false,
                                             convex_hull_indices,
                                             ground_contact_points);


        // check if check_pos is in hull
        bool center_in_hull = true;

        for (unsigned int i = 0; i < convex_hull_points.size(); ++i){
            if (i < convex_hull_points.size() - 1){
                if (sign(ccw(convex_hull_points.at(i), convex_hull_points.at(i+1), check_pos)) == -1){
                    center_in_hull = false;
                }
            }
        } // end for



        if (center_in_hull == true){
            ROS_INFO("check in hull is true");
            // The checkpos is in the convex hull. nothing needs to be changed
            break; // endwhile
        }

        // find supppolygon, if check_pos not in supppolygon do again with another supp p 1 TODO this is not tested, counter should not be more than 1
        counter++;
        ROS_INFO("%i iteration while loop", counter);


        if (counter > 1){
            ROS_INFO("ERROR -Supporting polygon after %i itereations not found // check_pos not in hull- ERROR", counter);
            break;
        }

        // checkpos is not in hull
        if (center_in_hull == false){
            ROS_INFO("check in hull is false");
            // find closest point to checkpos
            float dist = distanceXY(convex_hull_points.at(0), check_pos);
            int index_of_closest_point = 0;
            for (unsigned int i = 1; i < convex_hull_points.size(); ++i){
                float dist_to_check = distanceXY(convex_hull_points.at(i), check_pos);
                if (dist_to_check < dist){
                    dist = dist_to_check;
                    index_of_closest_point = i;
                }
            }
            support_point_1 = convex_hull_points.at(index_of_closest_point);
            convex_hull_indices.clear();
            ground_contact_points->clear();
        }


    } // endwhile



    //Compute and Draw Projected Robot

    //Compute points and normal of robot
    pcl::PointXYZ normal;
    pcl::PointXYZ robot_point_0;
    pcl::PointXYZ robot_point_1;
    pcl::PointXYZ robot_point_2;
    pcl::PointXYZ robot_point_3;
    pcl::PointXYZ robot_point_mid;
    pcl::PointXYZ center_of_mass_iterative;

    computeRobotPosition(support_point_1, support_point_2, support_point_3, p0, p1, p2, p3,
                         offset_CM,
                         normal,
                         robot_point_0, robot_point_1, robot_point_2, robot_point_3,
                         robot_point_mid, center_of_mass_iterative);

    // draw robot lines
    viewer.addLine(robot_point_0,robot_point_1,1.0,1.0,1.0,"f0");
    viewer.addLine(robot_point_1,robot_point_2,1.0,1.0,1.0,"f1");
    viewer.addLine(robot_point_2,robot_point_3,1.0,1.0,1.0,"f2");
    viewer.addLine(robot_point_3,robot_point_0,1.0,1.0,1.0,"f3");

    // render points
    /*  viewer.addSphere(support_point_1, 0.02,1,0,0, "sp1", viewport);
   viewer.addSphere(support_point_2, 0.02,0,1,0, "sp2", viewport);
   viewer.addSphere(support_point_3, 0.02,0,0,1, "sp3", viewport);
   for (unsigned int i = 0; i < cloud_positionRating2->size(); ++i){
       std::string name ="groundContactArea"+boost::lexical_cast<std::string>(i);
       viewer.addSphere(cloud_positionRating2->at(i), 0.01,0,1,1, name, viewport);
   }

    // draw center
    viewer.addSphere(check_pos, 0.03,1,0,0, "checkPosition", viewport);
    viewer.addSphere(robot_point_mid, 0.03,0,1,0, "proMidx", viewport);
    viewer.addSphere(robot_center_of_mass, 0.03,0,0,1, "CM", viewport);
    // draw normal
    const pcl::PointXYZ p_uff(addPoints(robot_point_mid, normal));
    viewer.addLine(robot_point_mid,p_uff,1.0,1.0,1.0,"fnormal");
    */



    //Compute Force Angle Stability Metric

    std::vector<float> rating =computeForceAngleStabilityMetric(center_of_mass_iterative,convex_hull_points);
    //  std::iterator max_it =std::max_element(rating.begin(),rating.end());


    for (unsigned int i=0; i<rating.size();++i)
    {

        // rating between convex_hull_point (i and i+1)

        float c =rating.at(i);
        std::string name ="convex_hull_rating"+boost::lexical_cast<std::string>(convex_hull_indices[i]);
        const pcl::PointXYZ p1(convex_hull_points[i].x,convex_hull_points[i].y,convex_hull_points[i].z);
        const pcl::PointXYZ p2(convex_hull_points[i+1].x,convex_hull_points[i+1].y,convex_hull_points[i+1].z);

        //ROS_INFO("RATING r: %f p1: %f %f %f p2:%f %f %f", c,p1.x,p1.y,p1.z,p2.x,p2.y,p2.z);
        ROS_INFO("Rating %f", rating.at(i));

        if (draw_convex_hull_first_polygon){
            if(c<invalid_rating)
                viewer.addLine(p1,p2,1,0,c/invalid_rating,name);
            else
                viewer.addLine(p1,p2,0,(c-invalid_rating)/invalid_rating,(invalid_rating*2-c)/invalid_rating,name);

            // draw convex hull (points only)
            std::string namech ="convexhullstart"+boost::lexical_cast<std::string>(i);
            viewer.addSphere(convex_hull_points[i], 0.025,0,1,0, namech,2 /*viewport*/);
        }
    }
    if (draw_convex_hull_first_polygon){
        viewer.addSphere(convex_hull_points[0], 0.025,1,0,0, "convexhullstart", 2 /*viewport*/);
    }

    for (unsigned int j = 0; j < rating.size(); j++){
        if (rating.at(j) < 1.0){
            unstable_axis += 1;
        }
    }

    if (tip_over_active){
        // iterative checking if still fine after flipping over invalid axis
        for (unsigned int i = 0; i < rating.size(); i++){
            if (rating.at(i) < invalid_rating){ // instabil
                ROS_INFO("Roboter kippt ueber Kante mit rating %f", rating.at(i));
                support_point_1 = convex_hull_points.at(i);
                support_point_2 = convex_hull_points.at(i+1);

                const pcl::PointXYZ tip_over_axis_vector= subtractPoints(support_point_2, support_point_1);
                pcl::PointXYZ tip_over_direction = crossProduct(tip_over_axis_vector, pcl::PointXYZ(0,0,1));


                bool point_evaluated = findSupportPoint(support_point_1,
                                                        tip_over_axis_vector,
                                                        (*cloud_positionRating),
                                                        tip_over_direction,
                                                        support_point_3);

                if (!point_evaluated){
                    break; // supp1 and 2 lie at the end of the area, no suppP3 could be found, invalide state, break for-loop and return initial max rating value
                }



                std::vector<unsigned int> convex_hull_indices_iterative;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_positionRating_iterative(new pcl::PointCloud<pcl::PointXYZ>());

                std::vector<pcl::PointXYZ> convex_hull_points_iterative = buildConvexHull(cloud_positionRating,
                                                                                          check_pos,
                                                                                          support_point_1,
                                                                                          support_point_2,
                                                                                          support_point_3,
                                                                                          true,
                                                                                          convex_hull_indices_iterative,
                                                                                          cloud_positionRating_iterative);


                float min_value = 100.0;
                unsigned int iterator = 0;
                for (unsigned int k = 0; k< convex_hull_points_iterative.size()-1; k++){
                    float value = vectorSimilarity(support_point_1, support_point_2,
                                                   convex_hull_points_iterative.at(k), convex_hull_points_iterative.at(k+1));
                    ROS_INFO("vector_similarity = %f", value);
                    if (value < min_value){
                        min_value = value;
                        iterator = k;
                    }
                }// TODO
                if (min_value == 100.0){
                    ROS_ERROR("[flor_terrain_classifier / model] after tipping over no edge could be removed from the rating");
                }
                // edge that must be removed is at position k in the vector

                convex_hull_points_iterative.erase(convex_hull_points_iterative.begin()+ convex_hull_points_iterative.size()-1);
                convex_hull_indices_iterative.erase(convex_hull_indices_iterative.begin() + convex_hull_indices_iterative.size() -1);
                for (unsigned int l = 0; l < iterator+1; l++){ //TODO looks right, maybe double check
                    // shift first of list to last of list
                    pcl::PointXYZ temp = convex_hull_points_iterative.at(0);
                    convex_hull_points_iterative.erase(convex_hull_points_iterative.begin()+0);
                    convex_hull_points_iterative.push_back(temp);

                    unsigned int temp_index = convex_hull_indices_iterative.at(0);
                    convex_hull_indices_iterative.erase((convex_hull_indices_iterative.begin() + 0));
                    convex_hull_indices_iterative.push_back(temp_index);

                }


                // THIS WAS DONE BEFORE AND SEEMED TO BE RIGHT THE UPPER IS NOT TESTED MUCH BUT IS NICER
                /* float similarity_distance = 0.04; // meter
             float similarity_angle = 10.0; // degree
             float sim_dist_add = 0.02;
             float sim_angle_add = 3.0;

             bool one_edge_removed = false;
             while (one_edge_removed == false){ // one edge must be removed
                for (unsigned int k = 0; k < convex_hull_points_iterative.size()-1; k++){
                    if (check_similar_vector(support_point_1, support_point_2,
                                            convex_hull_points_iterative.at(k), convex_hull_points_iterative.at(k+1),
                                            similarity_distance, similarity_angle)){
                        // put in right order and delete last element so a gap is made.
                        convex_hull_points_iterative.erase(convex_hull_points_iterative.begin()+ convex_hull_points_iterative.size()-1);
                        convex_hull_indices_iterative.erase(convex_hull_indices_iterative.begin() + convex_hull_indices_iterative.size() -1);
                        for (unsigned int l = 0; l < k+1; l++){ //TODO looks right, maybe double check
                            // shift first of list to last of list
                            pcl::PointXYZ temp = convex_hull_points_iterative.at(0);
                            convex_hull_points_iterative.erase(convex_hull_points_iterative.begin()+0);
                            convex_hull_points_iterative.push_back(temp);

                            unsigned int temp_index = convex_hull_indices_iterative.at(0);
                            convex_hull_indices_iterative.erase((convex_hull_indices_iterative.begin() + 0));
                            convex_hull_indices_iterative.push_back(temp_index);

                        }

                        one_edge_removed = true;
                        break; // only one edge can be removed, should not be a problem if hull is made smooth.
                    }
                }
                similarity_distance += sim_dist_add;
                similarity_angle += sim_angle_add;
             }
*/
                // draw supporting points
                viewer.addSphere(support_point_1, 0.02,1,0,0, "sp1it_tippedOver"+boost::lexical_cast<std::string>(i), viewport);
                viewer.addSphere(support_point_2, 0.02,0,1,0, "sp2it_tippedOver"+boost::lexical_cast<std::string>(i), viewport);
                viewer.addSphere(support_point_3, 0.02,0,0,1, "sp3it_tippedOver"+boost::lexical_cast<std::string>(i), viewport);
                if (draw_convex_hull_iterative_ground_points){
                    for (unsigned int j = 0; j < cloud_positionRating_iterative->size(); ++j){
                        std::string name ="groundContactAreait"+boost::lexical_cast<std::string>(j)+"tippedOver"+boost::lexical_cast<std::string>(i);
                        viewer.addSphere(cloud_positionRating_iterative->at(j), 0.01,0,1,1, name, viewport);
                    }
                }
                // draw robot lines
                /*
             viewer.addLine(robot_point_0,robot_point_1,1.0,1.0,1.0,"f0it" + i);
             viewer.addLine(robot_point_1,robot_point_2,1.0,1.0,1.0,"f1it" + i);
             viewer.addLine(robot_point_2,robot_point_3,1.0,1.0,1.0,"f2it" + i);
             viewer.addLine(robot_point_3,robot_point_0,1.0,1.0,1.0,"f3it" + i);*/
                // draw center
                if (i == 1){
                    viewer.addSphere(check_pos, 0.05,1,0,0, "checkPositionit", viewport);
                    viewer.addSphere(robot_point_mid, 0.05,0,1,0, "proMidxit", viewport);
                    viewer.addSphere(center_of_mass_iterative, 0.05,0,0,1, "CMit", viewport);
                    // draw normal
                    const pcl::PointXYZ p_uff(addPointVector(robot_point_mid, normal));
                    viewer.addLine(robot_point_mid,p_uff,1.0,1.0,1.0,"fnormalit");
                }


                //Compute Force Angle Stability Metric
                std::vector<float> rating_iterative =computeForceAngleStabilityMetric(center_of_mass_iterative,convex_hull_points_iterative);
                for (unsigned int k=0; k<rating_iterative.size();++k)
                {

                    // rating between convex_hull_point (i and i+1)
                    float c =rating_iterative.at(k);

                    std::string name ="convex_hull_rating_it"+boost::lexical_cast<std::string>(convex_hull_indices_iterative[k])+"tippedOver"+boost::lexical_cast<std::string>(i);
                    const pcl::PointXYZ p1(convex_hull_points_iterative[k].x,convex_hull_points_iterative[k].y,convex_hull_points_iterative[k].z);
                    const pcl::PointXYZ p2(convex_hull_points_iterative[k+1].x,convex_hull_points_iterative[k+1].y,convex_hull_points_iterative[k+1].z);

                    //ROS_INFO("RATING r: %f p1: %f %f %f p2:%f %f %f", c,p1.x,p1.y,p1.z,p2.x,p2.y,p2.z);
                    ROS_INFO("Rating %f", rating_iterative.at(k));


                    if (draw_convex_hull_iterative){

                        if(c<invalid_rating)
                            viewer.addLine(p1,p2,1,0,c/invalid_rating,name);
                        else
                            viewer.addLine(p1,p2,0,(c-invalid_rating)/invalid_rating,(invalid_rating*2-c)/invalid_rating,name);


                        // draw convex hull (points only)
                        std::string namech ="convexhullstart_it"+boost::lexical_cast<std::string>(k)+"tippedOver"+boost::lexical_cast<std::string>(i);
                        viewer.addSphere(convex_hull_points_iterative[k+1], 0.015,0,1,0, namech,2 /*viewport*/);
                    }

                }
                if (draw_convex_hull_iterative){
                    viewer.addSphere(convex_hull_points_iterative[0], 0.015,1,0,0, "convexhullstart_it_tippedOver"+boost::lexical_cast<std::string>(i), 2 /*viewport*/);
                }



                float min_rating_iterative =*std::min_element(rating_iterative.begin(),rating_iterative.end());

                // wenn gekippt und es hier stabiler ist, übernehme den schlechtesen Wert der gekippten Position an
                // der entsprechenden Stelle
                if (min_rating_iterative > rating.at(i)){
                    rating.at(i) = min_rating_iterative;
                }

            }
        }
    } // usetippingover end

    // draw groundpoints
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud_positionRating, "intensity");
    viewer.addPointCloud<pcl::PointXYZI>(cloud_positionRating,intensity_distribution, "positionRating_cloud", viewport);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "positionRating" + std::string("_edges"), viewport);

    position_rating = *std::min_element(rating.begin(),rating.end());
    ROS_INFO("position_rating = %f, contact_area not implemented, unstable_axis = %i", position_rating, unstable_axis);

    return true;
}

}
