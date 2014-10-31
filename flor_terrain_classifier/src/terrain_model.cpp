#include <flor_terrain_classifier/terrain_model.h>

namespace hector_terrain_model
{

TerrainModel::TerrainModel()
{
}

TerrainModel::TerrainModel(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    world_pcl_ptr= cloud.makeShared();

    // PARAMETER
    robot_length = 0.50; // [m] x for Obelix 0.54
    robot_width = 0.50; // [m] y for Obelix 0.56
    offset_CM = Eigen::Vector3f(0.0,0.0,0.12); // [m] offset of the center of mass
    minimum_distance = 0.08; // [m] default 0.8 minimum distance in which a support point can be found from the last one. causes problems, might not find supp3 even if it would be a valid polygon
    //invalid_rating = 0.3; // default: 0.3 theoretically 0
    //invalid_angle = 40; // [degree] highest considered stable angle
    delta_for_contact = 0.015; //  [m] +- delta for considering a ground point touching the robot
    tip_over = true;
    //smoothing the supporting polygon - important if tip_over is active.
    distance_smoothing = true;
    angle_smoothing = true;
    smooth_max_angle = 20.0;
    smooth_max_distance = 0.05;
#ifdef viewer_on
    draw_convex_hull_first_polygon = true;
    draw_convex_hull_ground_points = true;
    draw_convex_hull_iterative = true;
    draw_convex_hull_iterative_ground_points= true;
#endif
}

TerrainModel::~TerrainModel()
{

}

void TerrainModel::updateCloud(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    world_pcl_ptr= cloud.makeShared();
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

/*Eigen::Vector3f pointToEigen(pcl::PointXYZ p){
    return Eigen::Vector3f(p.x, p.y, p.z);
}*/

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

Eigen::Vector3f crossProductEigen(const pcl::PointXYZ& a,const  pcl::PointXYZ& b){
    float cx=a.y*b.z-a.z*b.y;
    float cy=a.z*b.x-a.x*b.z;
    float cz=a.x*b.y-a.y*b.x;
    return Eigen::Vector3f(cx,cy,cz);
}

// returns angle value in degree
float angleBetween(Eigen::Vector3f v1, Eigen::Vector3f v2){
    float acosvalue = dotproductEigen(v1,v2)/
            (sqrt(dotproductEigen(v1,v1)) * sqrt(dotproductEigen(v2,v2)));
    if (acosvalue > 0.9999999){
        return 0.0;
    }
    if (acosvalue < -0.999999){
        return 180.0;
    }
    float result = acos(acosvalue) * (180.0 / M_PI);

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


float angleToGround(const pcl::PointXYZ s1, const pcl::PointXYZ s2, const pcl::PointXYZ s3){
    Eigen::Vector3f normal = crossProductEigen(subtractPoints(s1, s2), subtractPoints(s1, s3));
    float angle = angleBetween(normal, Eigen::Vector3f(0.0, 0.0, 1.0));
    if (angle > 90.0)
        angle = 180.0 - angle;
    return angle;
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
    unsigned int i=0;
    unsigned int current_best;
    while (true)
    {
       if(std::find(convex_hull_indices.begin(), convex_hull_indices.end(), point_on_hull) != convex_hull_indices.end())
        {
           ROS_WARN("convex_hull_comp is in endless loop quickfix");
           // first point is also last point
           convex_hull_indices.push_back(convex_hull_indices.at(0));
           break;
       }
           convex_hull_indices.push_back(point_on_hull);

        current_best=0;
        //if((0==i)&&(0==point_on_hull)) current_best=1;
        for(unsigned int current_candidate=1; current_candidate<cloud_2d.size();++current_candidate)
        {           
            unsigned int lastHullElement=convex_hull_indices.at(i);
            if(   (lastHullElement != current_best)
                 && (lastHullElement != current_candidate)
                 && (current_candidate != current_best))
            {
                pcl::PointXYZ& p0 =cloud_2d.at(convex_hull_indices.at(i)); // aktuell letzter hullpoint
                pcl::PointXYZ& p1 =cloud_2d.at(current_best); // kandidat ( aktuell bester nächster )
                pcl::PointXYZ& p2 =cloud_2d.at(current_candidate); // kandidat der gegen den aktuell besten verglichen wird)
                float ccw_f=ccw(p0,p1,p2);
                bool isleft=(ccw_f < 0);
                if(current_best==point_on_hull // for 2nd point only
                        || isleft) // candidate is more left
                {
                    current_best=current_candidate;
                }

                else if(ccw_f == 0) // straight or backwards
                {
                    pcl::PointXYZ& p_before = cloud_2d.at(convex_hull_indices.at(i));
                    pcl::PointXYZ direction_before = subtractPoints(p0, p_before);
                    pcl::PointXYZ direction_current = subtractPoints(p0, p2);
                    if (dotProduct(direction_before, direction_current) > 0)// selbe richtung vom kandidat und dem vorhergegangenen
                    {
                        float dist_endpoint = distanceXY(p0, p1);
                        float dist_j = distanceXY(p0, p2);
                        if (dist_j < dist_endpoint)
                        {
                            current_best = current_candidate;
                        }
                    }
                }
            }
            else
            {
                if(lastHullElement == current_best)
                {
                    current_best++;
                }
               // ROS_WARN("CH same id %i %i %i .",lastHullElement,current_best,current_candidate);

            }
        }

        i++;
        point_on_hull=current_best;
        if(current_best==convex_hull_indices.at(0))
        {
            // first point is also last point
            convex_hull_indices.push_back(convex_hull_indices.at(0));
            break;
        }

        if (i > 2*cloud_2d.size()){
            ROS_WARN("convex_hull_comp is in endless loop");
        }

    }

}

// computes the minimum Rating if standing flat on the ground
float TerrainModel::minPosRating(){
    pcl::PointXYZ p0 = pcl::PointXYZ(0.0, 0.0, 0.0);
    pcl::PointXYZ p1 = pcl::PointXYZ(robot_length, 0.0, 0.0);
    pcl::PointXYZ p2 = pcl::PointXYZ(robot_length, robot_width, 0.0);
    pcl::PointXYZ p3 = pcl::PointXYZ(0.0, robot_width, 0.0);

    pcl::PointXYZ mid = pcl::PointXYZ(robot_length / 2.0, robot_width / 2.0, 0.0);
    pcl::PointXYZ center_mass = addPointVector(mid, offset_CM);

    std::vector<pcl::PointXYZ> flat_points;
    flat_points.push_back(p0);
    flat_points.push_back(p1);
    flat_points.push_back(p2);
    flat_points.push_back(p3);
    flat_points.push_back(p0);



    std::vector<float> min_pos_rating = computeForceAngleStabilityMetric(center_mass, flat_points);
    float min = min_pos_rating.at(0);

    if (min_pos_rating.at(1) < min)
        min = min_pos_rating.at(1);

    if (min_pos_rating.at(2) < min)
        min = min_pos_rating.at(2);

    if (min_pos_rating.at(3) < min)
        min = min_pos_rating.at(3);

    return min;
}


//As proposed in "Modeling the manipulator and flipper pose effects on tip over stability of a tracked mobile manipulator" by Chioniso Dube
// a low number is not stable. a high number is stable.
std::vector<float> TerrainModel::computeForceAngleStabilityMetric(const pcl::PointXYZ& center_of_mass_pcl, std::vector<pcl::PointXYZ>& convex_hull_points_pcl)
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
                                    pcl::PointXYZ& support_point){

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
    pcl::PointXYZ v1 = tip_over_direction;


    //states the direction of the points if on one side of the tip_over_axis, or on the other
    // 1 = counterclockwise, -1 = clockwise
    const int directionccw = sign(ccw (tip_over_axis_point,
                                       addPointVector(tip_over_axis_point, tip_over_axis_vector),
                                       addPointVector(tip_over_axis_point, tip_over_direction)));


    for(unsigned int i=0; i<cloud_projected.size();++i)
    {
        pcl::PointXYZ &p_pro= cloud_projected.at(i);
        const pcl::PointXYZ v2 = pcl::PointXYZ(p_pro.x-tip_over_axis_point.x,p_pro.y-tip_over_axis_point.y,p_pro.z-tip_over_axis_point.z);
        //float angle= acos( dotProduct(v1,v2)/sqrt(dotProduct(v1,v1)*dotProduct(v2,v2))); // ERROR???? sqrt(dotproduct) * sqrt(dotproduct)

        Eigen::Vector3f v1e(v1.x,v1.y,v1.z);
        Eigen::Vector3f v2e(v2.x,v2.y,v2.z);

        float angle = angleBetween(v1e,v2e);// angle * 360.0/(2.0*M_PI);
        if(angle>180.0)
            ROS_ERROR("[Terrain_model] angle = %f", angle);

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
        else
        {

            //ROS_INFO("find point angle %f minangle %f side b %b side %i ccw %i", angle, min_angle,(side == directionccw),side,ccw );
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



    const pcl::PointXYZ final_normal= crossProduct(pcl::PointXYZ(support_point_1.x-support_point_2.x,support_point_1.y-support_point_2.y,support_point_1.z-support_point_2.z),
                                                   pcl::PointXYZ(support_point_1.x-support_point_3.x,support_point_1.y-support_point_3.y,support_point_1.z-support_point_3.z));


    //Find ground contact points
    for (unsigned int i = 0; i < cloud_positionRating->size(); i++)
    {
        pcl::PointXYZI& p = cloud_positionRating->at(i);
        const float dist = planeDistance(pcl::PointXYZ(p.x,p.y,p.z),final_normal,support_point_1);
        if(fabs(dist) < delta_for_contact)
        {
            if (iterative == false){
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
        //ROS_INFO("distance smoothing");
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

        //ROS_INFO("end distance smoothing");
    }

    // angle smoothing
    if (angle_smoothing){

        //ROS_INFO("angle smoothing");
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
        //ROS_INFO("end angle smoothing");
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

    // ERROR ? ACHTUNG EVENTUELL FLASCH RUM, DA ICH NICHT WEISS WIE RUM DER ROBOTER STEHT // EGAL, wenn offset_CM nur Werte in z hat
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

void TerrainModel::computeRobotEdgePoints(const pcl::PointXYZ check_pos, const float orientation,
                            pcl::PointXYZ& p0, pcl::PointXYZ& p1, pcl::PointXYZ& p2, pcl::PointXYZ& p3){

    p0=pcl::PointXYZ(+ 0.5*robot_length, + 0.5*robot_width, 0);
    p0 = rotatePoint(p0, orientation);
    p0 = addPointVector(p0, check_pos);

    p1=pcl::PointXYZ(- 0.5*robot_length, + 0.5*robot_width, 0);
    p1 = rotatePoint(p1, orientation);
    p1 = addPointVector(p1, check_pos);

    p2=pcl::PointXYZ(- 0.5*robot_length, - 0.5*robot_width, 0);
    p2 = rotatePoint(p2, orientation);
    p2 = addPointVector(p2, check_pos);

    p3=pcl::PointXYZ(+ 0.5*robot_length, - 0.5*robot_width, 0);
    p3 = rotatePoint(p3, orientation);
    p3 = addPointVector(p3, check_pos);
}

void TerrainModel::fillRobotPointcloud(const pcl::PointXYZ& p0, const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3, unsigned int& highest_Point_idx){

    robot_pcl.reset(new pcl::PointCloud<pcl::PointXYZI>());

    unsigned int n_counter=0;

    std::vector<pcl::PointXYZ> robot_points;
    robot_points.push_back(p0);
    robot_points.push_back(p1);
    robot_points.push_back(p2);
    robot_points.push_back(p3);

    float maxX = p0.x;
    float minX = p0.x;
    float maxY = p0.y;
    float minY = p0.y;

    for (unsigned int i = 0; i < robot_points.size(); i++){
        pcl::PointXYZ cp = robot_points.at(i); // current point
        if (cp.x > maxX)
            maxX = cp.x;
        if (cp.x < minX)
            minX = cp.x;
        if (cp.y > maxY)
            maxY = cp.y;
        if (cp.y < minY)
            minY = cp.y;
    }

    bool hull_cpp= (ccw(p0,p1,p2)<0);
    for (unsigned int i = 0; i < world_pcl_ptr->size(); i++)
    {
        const  pcl::PointXYZ &pp= world_pcl_ptr->at(i); // teuer?

        if ((pp.x > minX && pp.x < maxX && pp.y > minY && pp.y < maxY) == false)
            continue;

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
            robot_pcl->push_back(p);

            if(n_counter==0) highest_Point_idx=1;
            else if(p.z>robot_pcl->at(highest_Point_idx).z) highest_Point_idx=n_counter;
            ++n_counter;
        }
    }


}


// -------------------------------------------------------------------------------------------------//
// -------------------------------------------------------------------------------------------------//
//-------------------------------------COMPUTE POSITION RATING--------------------------------------//
//--------------------------------------------------------------------------------------------------//
// -------------------------------------------------------------------------------------------------//

// orientation in radiants
bool TerrainModel::computePositionRating(const pcl::PointXYZ& check_pos,
                                         const float orientation,
                                         pcl::PointXYZ& robot_point_center,
                                         pcl::PointXYZ& robot_point_0,
                                         pcl::PointXYZ& robot_point_1,
                                         pcl::PointXYZ& robot_point_2,
                                         pcl::PointXYZ& robot_point_3,
                                         float &position_rating, // after tipping over
                                         //float contact_area, // of the first polygon // TODO
                                         int &unstable_axis // of the first polygon
                   #ifdef viewer_on
                                         , pcl::visualization::PCLVisualizer &viewer,
                                         int viewport
                   #endif
                                         )
{    
    // times for debug / to see the speed of the algorithm
    double time_start_robotplcfind;
    double time_start_findSupPoints23;
    double time_start_computeHull;
    double time_start_positionRating;
    double time_duration_robotpclfind;
    double time_duration_findSupPoints23;
    double time_duration_computeHull;
    double time_duration_positionRating;

    position_rating = 0.0;
    unstable_axis = 0;

    // Points at the edges of the robot (flat ground)
    pcl::PointXYZ p0, p1, p2, p3; // Edge Points
    computeRobotEdgePoints(check_pos, orientation, p0,p1,p2,p3);

    time_start_robotplcfind =ros::Time::now().toNSec();


    // fill the robot_pcl
    unsigned int highest_Point_idx;
    fillRobotPointcloud(p0, p1, p2, p3, highest_Point_idx);

    if(robot_pcl->size()==0) {
        ROS_WARN("[flor terrain classifier] robot_pcl size is 0. this could be due to planning into space without points.");
        return false;
    }

    time_duration_robotpclfind = (ros::Time::now().toNSec() - time_start_robotplcfind)/1000;
   ////ROS_INFO("time for robot cloud compute [mikrosec] = %i", (int)time_duration_robotpclfind);

    //ROS_INFO("cloud position rating size = %i", cloud_positionRating->size());

    pcl::PointXYZI &p_max= robot_pcl->at(highest_Point_idx);//highest point
    int support_point_1_idx;
    float min_dist=-1.0;
    //choose highest point (+-delta), closest to CoM
    for (unsigned int i = 0; i < robot_pcl->size(); i++)
    {
        pcl::PointXYZI& p = robot_pcl->at(i);
        if(((p.z-p_max.z)<0.03)&&((p.z-p_max.z)>-0.03))
        {
            float dist=sqrt((p.x-check_pos.x)*(p.x-check_pos.x)+(p.y-check_pos.y)*(p.y-check_pos.y));
            if(min_dist<0.0 || dist<min_dist)
            {
                min_dist=dist;
                support_point_1_idx=i;
            }

        }
    }
    pcl::PointXYZ support_point_1 = pcl::PointXYZ(robot_pcl->at(support_point_1_idx).x,robot_pcl->at(support_point_1_idx).y,robot_pcl->at(support_point_1_idx).z);


    // is used after while loop, but initiated in it.
    pcl::PointXYZ support_point_2;
    pcl::PointXYZ support_point_3;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_contact_points(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<unsigned int> convex_hull_indices;
    std::vector<pcl::PointXYZ> convex_hull_points;

    int counter = 0;

    while (true){ // find SupportPoints ; run only once if center is in hull


        time_start_findSupPoints23 =ros::Time::now().toNSec();
        // find supp P 2
        const pcl::PointXYZ tip_over_axis_point = support_point_1;
        pcl::PointXYZ tip_over_axis_vector = (crossProduct(pcl::PointXYZ(support_point_1.x-check_pos.x,support_point_1.y-check_pos.y,0),pcl::PointXYZ(0,0,1)));
        pcl::PointXYZ tip_over_direction;

        float delta = 0.0000001;
        if (tip_over_axis_vector.x < delta && tip_over_axis_vector.y < delta && tip_over_axis_vector.z < delta){

            tip_over_axis_vector = pcl::PointXYZ(1.0, 0.0, 0.0); // this is random.
            tip_over_direction = pcl::PointXYZ(0.0, 1.0, 0.0); // 90° from vector

        }
        else{
            tip_over_direction = pcl::PointXYZ(check_pos.x - support_point_1.x, check_pos.y - support_point_1.y, 0);
        }

        //ROS_INFO("tip_over_direction x,y,z = %f, %f, %f", tip_over_direction.x, tip_over_direction.y, tip_over_direction.z);
       //ROS_INFO("tip_over_axis_vector x,y,z = %f, %f, %f", tip_over_axis_vector.x, tip_over_axis_vector.y, tip_over_axis_vector.z);



        bool point_evaluated = findSupportPoint(tip_over_axis_point,
                                                tip_over_axis_vector,
                                                (*robot_pcl),
                                                tip_over_direction,
                                                support_point_2);
        if (!point_evaluated){
            ROS_WARN("[terrain_model] no support_point found 2..., counter = %i ", counter);
            //ROS_INFO("checkpos x = %f, y 0 %f, supportPoint1 x = %f, y = %f", check_pos.x, check_pos.y, support_point_1.x, support_point_1.y);
            return false;
        }

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
                                           (*robot_pcl),
                                           tip_over_direction_3,
                                           support_point_3);
        if (!point_evaluated){
           //ROS_INFO("[terrain_model::compute_position_rating] no support_point found 3.");
            return false;
        }

        time_duration_findSupPoints23 = (ros::Time::now().toNSec() - time_start_findSupPoints23)/1000;
       ////ROS_INFO("time for finding support Points 2 and 3 compute [mikrosec] = %i", (int)time_duration_findSupPoints23);

        time_start_computeHull =ros::Time::now().toNSec();

        convex_hull_points = buildConvexHull(robot_pcl,
                                             check_pos,
                                             support_point_1,
                                             support_point_2,
                                             support_point_3,
                                             false,
                                             convex_hull_indices,
                                             ground_contact_points);

        time_duration_computeHull = (ros::Time::now().toNSec() - time_start_computeHull)/1000;
      // //ROS_INFO("time for computeHull [mikrosec] = %i", (int)time_duration_computeHull);


#ifdef viewer_on // draw ground contact
        if (draw_convex_hull_ground_points){
            for (unsigned int j = 0; j < ground_contact_points->size(); ++j){
                std::string name ="groundContactArea"+boost::lexical_cast<std::string>(j);
                viewer.addSphere(ground_contact_points->at(j), 0.01,0,1,1, name, viewport);
            }
        }
#endif

        // check if check_pos is in hull
        bool center_in_hull = true;


        //ROS_INFO("Elements in convex hull %i", convex_hull_points.size());
        for (unsigned int i = 0; i < convex_hull_points.size(); ++i){
            if (i < convex_hull_points.size() - 1){
                if (sign(ccw(convex_hull_points.at(i), convex_hull_points.at(i+1), check_pos)) == -1){
                    center_in_hull = false;
                }
            }
        } // end for



        if (center_in_hull == true){
            //ROS_INFO("check in hull is true");
            // The checkpos is in the convex hull. nothing needs to be changed
            break; // endwhile
        }

        // find supppolygon, if check_pos not in supppolygon do again with another supp p 1 TODO this is not tested, counter should not be more than 1
        counter++;
        //ROS_INFO("%i iteration while loop", counter);


        if (counter > 1){
            //ROS_INFO("ERROR -Supporting polygon after %i itereations not found // check_pos not in hull- ERROR", counter);
            break;
        }

        // checkpos is not in hull
        if (center_in_hull == false){
            //ROS_INFO("check in hull is false");
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

    float angle_of_area = angleToGround(support_point_1, support_point_2, support_point_3);
    if (angle_of_area > invalid_angle)
        return false;


    //Compute and Draw Projected Robot

    //Compute points and normal of robot
    pcl::PointXYZ normal;
    pcl::PointXYZ center_of_mass;

    computeRobotPosition(support_point_1, support_point_2, support_point_3, p0, p1, p2, p3,
                         offset_CM,
                         normal,
                         robot_point_0, robot_point_1, robot_point_2, robot_point_3,
                         robot_point_center, center_of_mass);
#ifdef viewer_on
    // draw robot lines
    viewer.addLine(robot_point_0,robot_point_1,1.0,1.0,1.0,"f0");
    viewer.addLine(robot_point_1,robot_point_2,1.0,1.0,1.0,"f1");
    viewer.addLine(robot_point_2,robot_point_3,1.0,1.0,1.0,"f2");
    viewer.addLine(robot_point_3,robot_point_0,1.0,1.0,1.0,"f3");
#endif

    //Compute Force Angle Stability Metric
    time_start_positionRating =ros::Time::now().toNSec();

    std::vector<float> rating =computeForceAngleStabilityMetric(center_of_mass,convex_hull_points);
    //  std::iterator max_it =std::max_element(rating.begin(),rating.end());

    time_duration_positionRating = (ros::Time::now().toNSec() - time_start_positionRating)/1000;
   ////ROS_INFO("time for rating [mikrosec] = %i", (int)time_duration_positionRating);



#ifdef viewer_on
    for (unsigned int i=0; i<rating.size();++i)
    {

        // rating between convex_hull_point (i and i+1)

        float c =rating.at(i);
        std::string name ="convex_hull_rating"+boost::lexical_cast<std::string>(convex_hull_indices[i]);
        const pcl::PointXYZ p1(convex_hull_points[i].x,convex_hull_points[i].y,convex_hull_points[i].z);
        const pcl::PointXYZ p2(convex_hull_points[i+1].x,convex_hull_points[i+1].y,convex_hull_points[i+1].z);

        ROS_INFO("RATING r: %f p1: %f %f %f p2:%f %f %f", c,p1.x,p1.y,p1.z,p2.x,p2.y,p2.z);
        //ROS_INFO("Rating %f", rating.at(i));

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

        viewer.addSphere(convex_hull_points[0], 0.025,1,0,0, "convexhullstart", 2 /*viewport*/);
        viewer.addSphere(check_pos, 0.05,1,0,0, "checkPosition", viewport);
        viewer.addSphere(robot_point_mid, 0.05,0,1,0, "proMidx", viewport);
        viewer.addSphere(center_of_mass, 0.05,0,0,1, "CM", viewport);


#endif

    for (unsigned int j = 0; j < rating.size(); j++){
        if (rating.at(j) < 1.0){
            unstable_axis += 1;
        }
    }

    // -------------------------------------------------------------------------//
    // -----------------------TIP OVER------------------------------------------//
    // -------------------------------------------------------------------------//

    if (tip_over){
        // iterative checking if still fine after flipping over invalid axis
        for (unsigned int i = 0; i < rating.size(); i++){
            if (rating.at(i) < invalid_rating){ // instabil
                #ifdef viewer_on
                    ROS_INFO("Roboter kippt ueber Kante mit rating %f", rating.at(i));
                #endif
                support_point_1 = convex_hull_points.at(i);
                support_point_2 = convex_hull_points.at(i+1);

                const pcl::PointXYZ tip_over_axis_vector= subtractPoints(support_point_2, support_point_1);
                pcl::PointXYZ tip_over_direction = crossProduct(tip_over_axis_vector, pcl::PointXYZ(0,0,1));


                bool point_evaluated = findSupportPoint(support_point_1,
                                                        tip_over_axis_vector,
                                                        (*robot_pcl),
                                                        tip_over_direction,
                                                        support_point_3);

                if (!point_evaluated){
                    break; // supp1 and 2 lie at the end of the area, no suppP3 could be found, invalide state, break for-loop and return initial max rating value
                }

                float angle_of_area_it = angleToGround(support_point_1, support_point_2, support_point_3);
                if (angle_of_area_it > invalid_angle){
                    ROS_INFO("after tipping over, angle would be too high: %f", angle_of_area_it);
                    break;
                }



                std::vector<unsigned int> convex_hull_indices_iterative;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_positionRating_iterative(new pcl::PointCloud<pcl::PointXYZ>());

                std::vector<pcl::PointXYZ> convex_hull_points_it = buildConvexHull(robot_pcl,
                                                                                          check_pos,
                                                                                          support_point_1,
                                                                                          support_point_2,
                                                                                          support_point_3,
                                                                                          true,
                                                                                          convex_hull_indices_iterative,
                                                                                          cloud_positionRating_iterative);

                pcl::PointXYZ normal_it;
                pcl::PointXYZ robot_point_0_it;
                pcl::PointXYZ robot_point_1_it;
                pcl::PointXYZ robot_point_2_it;
                pcl::PointXYZ robot_point_3_it;
                pcl::PointXYZ robot_point_mid_it;
                pcl::PointXYZ center_of_mass_it;

                computeRobotPosition(support_point_1, support_point_2, support_point_3, p0, p1, p2, p3,
                                     offset_CM,
                                     normal_it,
                                     robot_point_0_it, robot_point_1_it, robot_point_2_it, robot_point_3_it,
                                     robot_point_mid_it, center_of_mass_it);

                float min_value = 100.0;
                unsigned int iterator = 0;
                for (unsigned int k = 0; k< convex_hull_points_it.size()-1; k++){
                    float value = vectorSimilarity(support_point_1, support_point_2,
                                                   convex_hull_points_it.at(k), convex_hull_points_it.at(k+1));
                    //ROS_INFO("vector_similarity = %f", value);
                    if (value < min_value){
                        min_value = value;
                        iterator = k;
                    }
                }// TODO
                if (min_value == 100.0){
                    ROS_ERROR("[flor_terrain_classifier / model] after tipping over no edge could be removed from the rating");
                }
                // edge that must be removed is at position k in the vector

                convex_hull_points_it.erase(convex_hull_points_it.begin()+ convex_hull_points_it.size()-1);
                convex_hull_indices_iterative.erase(convex_hull_indices_iterative.begin() + convex_hull_indices_iterative.size() -1);
                for (unsigned int l = 0; l < iterator+1; l++){ //TODO looks right, maybe double check
                    // shift first of list to last of list
                    pcl::PointXYZ temp = convex_hull_points_it.at(0);
                    convex_hull_points_it.erase(convex_hull_points_it.begin()+0);
                    convex_hull_points_it.push_back(temp);

                    unsigned int temp_index = convex_hull_indices_iterative.at(0);
                    convex_hull_indices_iterative.erase((convex_hull_indices_iterative.begin() + 0));
                    convex_hull_indices_iterative.push_back(temp_index);

                }
#ifdef viewer_on
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

                // draw center
                if (i == 0){
                    viewer.addSphere(robot_point_mid_it, 0.04,0,1,0, "proMidxit", viewport);
                    viewer.addSphere(center_of_mass_it, 0.04,0,0,1, "CMit", viewport);
                    // draw normal
                    const pcl::PointXYZ p_uff(addPointVector(robot_point_mid, normal));
                    viewer.addLine(robot_point_mid,p_uff,1.0,1.0,1.0,"fnormalit");
                }
#endif


                //Compute Force Angle Stability Metric
                std::vector<float> rating_iterative =computeForceAngleStabilityMetric(center_of_mass_it,convex_hull_points_it);
                for (unsigned int k=0; k<rating_iterative.size();++k)
                {

                    // rating between convex_hull_point (i and i+1)
                    //float c =rating_iterative.at(k);

                    std::string name ="convex_hull_rating_it"+boost::lexical_cast<std::string>(convex_hull_indices_iterative[k])+"tippedOver"+boost::lexical_cast<std::string>(i);
                    const pcl::PointXYZ p1(convex_hull_points_it[k].x,convex_hull_points_it[k].y,convex_hull_points_it[k].z);
                    const pcl::PointXYZ p2(convex_hull_points_it[k+1].x,convex_hull_points_it[k+1].y,convex_hull_points_it[k+1].z);

                    //ROS_INFO("RATING r: %f p1: %f %f %f p2:%f %f %f", c,p1.x,p1.y,p1.z,p2.x,p2.y,p2.z);
#ifdef viewer_on
                    ROS_INFO("Rating %f", rating_iterative.at(k));
                    if (draw_convex_hull_iterative){

                        if(c<invalid_rating)
                            viewer.addLine(p1,p2,1,0,c/invalid_rating,name);
                        else
                            viewer.addLine(p1,p2,0,(c-invalid_rating)/invalid_rating,(invalid_rating*2-c)/invalid_rating,name);


                        // draw convex hull (points only)
                        std::string namech ="convexhullstart_it"+boost::lexical_cast<std::string>(k)+"tippedOver"+boost::lexical_cast<std::string>(i);
                        viewer.addSphere(convex_hull_points_it[k+1], 0.015,0,1,0, namech,2 /*viewport*/);
                    }
#endif
                }
#ifdef viewer_on
                if (draw_convex_hull_iterative){
                    viewer.addSphere(convex_hull_points_it[0], 0.015,1,0,0, "convexhullstart_it_tippedOver"+boost::lexical_cast<std::string>(i), 2 /*viewport*/);
                }
#endif


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
#ifdef viewer_on
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(robot_pcl, "intensity");
    viewer.addPointCloud<pcl::PointXYZI>(robot_pcl,intensity_distribution, "positionRating_cloud", viewport);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "positionRating" + std::string("_edges"), viewport);
#endif
    position_rating = *std::min_element(rating.begin(),rating.end());
    //ROS_INFO("CPR done: position_rating = %f, unstable_axis = %i", position_rating, unstable_axis);

    double time_together = time_duration_robotpclfind +
            time_duration_findSupPoints23 +
            time_duration_computeHull +
            time_duration_positionRating;
   ////ROS_INFO("time for evaluated time [mikrosec]", (int)time_together);

    return true;
}

}
