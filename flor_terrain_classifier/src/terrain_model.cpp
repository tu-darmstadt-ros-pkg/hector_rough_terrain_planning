#include <flor_terrain_classifier/terrain_model.h>
#include <flor_terrain_classifier/pcl_math_utils.h>

#include <pcl/surface/convex_hull.h>

#include <algorithm>
#include <iostream>
#include <vector>
//#define time_debug;


#define MY_EPS 1e-6
namespace hector_terrain_model
{

TerrainModel::TerrainModel()
{
}

TerrainModel::TerrainModel(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    world_pcl_ptr= cloud.makeShared();

    // PARAMETER
    robot_length = 0.55; // [m] x for Obelix 0.54
    robot_width = 0.40; // [m] y for Obelix 0.56
    offset_CM = Eigen::Vector3f(0.0,0.0,0.12); // [m] offset of the center of mass
    minimum_distance = 0.08; // [m] default 0.8 minimum distance in which a support point can be found from the last one. causes problems, might not find supp3 even if it would be a valid polygon
    invalid_rating = 0.3; // default: 0.3 theoretically 0
    invalid_angle = 35; // [degree] highest considered stable angle
    delta_for_contact = 0.015; //  [m] +- delta for considering a ground point touching the robot
    first_SP_around_mid = false; // true means first polygon must contain the checkpos in it
    tip_over = false;
    check_each_axis = true; // check each axis after tipping over to maybe improve the rating value
    //smoothing the supporting polygon - important if tip_over is active.
    distance_smoothing = true;
    angle_smoothing = true;
    smooth_max_angle = 20.0;
    smooth_max_distance = 0.045; // not needed for octomap with gridsize 5 cm

    draw_convex_hull_first_polygon = true;
    draw_convex_hull_ground_points = true;
    draw_convex_hull_iterative = true;
    draw_convex_hull_iterative_ground_points= true;

}

TerrainModel::~TerrainModel()
{

}

void TerrainModel::updateCloud(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    world_pcl_ptr= cloud.makeShared();
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


// return the value of the least stable axis for the robot standing on flat ground
float TerrainModel::optimalPossiblePositionRating(){
    pcl::PointXYZ projected_robot_p0 = pcl::PointXYZ(0.0, 0.0, 0.0);
    pcl::PointXYZ projected_robot_p1 = pcl::PointXYZ(robot_length, 0.0, 0.0);
    pcl::PointXYZ projected_robot_p2 = pcl::PointXYZ(robot_length, robot_width, 0.0);
    pcl::PointXYZ projected_robot_p3 = pcl::PointXYZ(0.0, robot_width, 0.0);

    pcl::PointXYZ projected_robot_center = pcl::PointXYZ(robot_length / 2.0, robot_width / 2.0, 0.0);
    pcl::PointXYZ center_mass = addPointVector(projected_robot_center, offset_CM);

    std::vector<pcl::PointXYZ> projected_robot_hull;
    projected_robot_hull.push_back(projected_robot_p0);
    projected_robot_hull.push_back(projected_robot_p1);
    projected_robot_hull.push_back(projected_robot_p2);
    projected_robot_hull.push_back(projected_robot_p3);
    projected_robot_hull.push_back(projected_robot_p0);

    std::vector<float> min_pos_rating = computeForceAngleStabilityMetric(center_mass, projected_robot_hull);
    std::vector<float>::iterator it_res = std::min_element(min_pos_rating.begin(), min_pos_rating.end());
    return *it_res;
}


//As proposed in "Modeling the manipulator and flipper pose effects on tip over stability of a tracked mobile manipulator" by Chioniso Dube
// a low values indicated an instable condition, a high number inidicates a stable condition, 0 is the physical tip over value
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
        if(p.size()==1)
            p_i1 = p.at(0);
        else
            p_i1 = p.at(i+1);
        const Eigen::Vector3f ai=(p_i1-p.at(i)).normalized();
        const Eigen::Matrix3f ident_mat =Eigen::Matrix3f::Identity();
        const Eigen::Vector3f li=(ident_mat-ai*ai.transpose())*(p_i1-center_of_mass);
        const Eigen::Vector3f fi=(ident_mat-ai*ai.transpose())*f_r;
        const Eigen::Vector3f li_norm=li.normalized();
        const Eigen::Vector3f fi_norm=fi.normalized();
        const Eigen::Vector3f di= (-li)+(li.dot(fi_norm))*fi_norm;
        const float theta_i=acos(li_norm.dot(fi_norm));
        const float sigma_i=((fi_norm.cross(li_norm)).dot(ai))?1.0:-1.0 ;
        const float beta=sigma_i*theta_i*di.norm()*f_r.norm();
        res.push_back(beta);
    }

    return res;
}



// evaluates the second or third point of the supporting plane
// if false is returned, there could not be a point found.
bool TerrainModel::findSupportPoint(const pcl::PointXYZ& tip_over_axis_point,
                                    const pcl::PointXYZ& tip_over_axis_vector,
                                    const pcl::PointCloud<pcl::PointXYZI>& pointcloud_robot,
                                    const pcl::PointXYZ& tip_over_direction,
                                    pcl::PointXYZ& support_point)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_projected;
    cloud_projected.resize(pointcloud_robot.size());
    for(unsigned int i=0; i<cloud_projected.size();++i)
    {
        pcl::PointXYZ &p_pro= cloud_projected.at(i);
        const pcl::PointXYZI &p_pos= pointcloud_robot.at(i);
        p_pro.x=p_pos.x;
        p_pro.y=p_pos.y;
        p_pro.z=p_pos.z;
        cloud_projected.at(i)=planeProjection(p_pro,tip_over_axis_vector,tip_over_axis_point);
    }

    float min_angle=360.0;
    int min_angle_idx = -1;

    //states the direction of the points if on one side of the tip_over_axis, or on the other
    // 1 = counterclockwise, -1 = clockwise
    const int orientation_to_axis = sign(ccw (tip_over_axis_point,
                                       addPointVector(tip_over_axis_point, tip_over_axis_vector),
                                       addPointVector(tip_over_axis_point, tip_over_direction)));

    for(unsigned int i=0; i<cloud_projected.size();++i)
    {
        pcl::PointXYZ &p_pro= cloud_projected.at(i);
        const pcl::PointXYZ axis_to_projected_point = pcl::PointXYZ(p_pro.x-tip_over_axis_point.x,p_pro.y-tip_over_axis_point.y,p_pro.z-tip_over_axis_point.z);
        //float angle= acos( dotProduct(v1,v2)/sqrt(dotProduct(v1,v1)*dotProduct(v2,v2))); // ERROR???? sqrt(dotproduct) * sqrt(dotproduct)

        Eigen::Vector3f v1e(tip_over_direction.x,tip_over_direction.y,tip_over_direction.z);
        Eigen::Vector3f v2e(axis_to_projected_point.x,axis_to_projected_point.y,axis_to_projected_point.z);

        float angle = angleBetween(v1e,v2e); //in degree
        if(angle > 180.0)
            ROS_ERROR("[Terrain_model] angle>180 = %f", angle);
        if (angle > 90.0) angle = 180.0-angle;

        // which side to the axis
        pcl::PointXYZ ps=pcl::PointXYZ(pointcloud_robot.at(i).x, pointcloud_robot.at(i).y, pointcloud_robot.at(i).z);
        const int side = sign(ccw(tip_over_axis_point,
                                  addPointVector(tip_over_axis_point, tip_over_axis_vector),
                                  ps));

        if (angle < min_angle && side == orientation_to_axis)
        {
            pcl::PointXYZ current = pcl::PointXYZ(pointcloud_robot.at(i).x,pointcloud_robot.at(i).y,pointcloud_robot.at(i).z);
            if (distancePointStraight(tip_over_axis_point, tip_over_axis_vector, current) > minimum_distance)
            {
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

    support_point=pcl::PointXYZ(pointcloud_robot.at(min_angle_idx).x,pointcloud_robot.at(min_angle_idx).y,pointcloud_robot.at(min_angle_idx).z);
    return true;
}



/* builds the convex hull with the given supportpoints */
std::vector<pcl::PointXYZ> TerrainModel:: buildConvexHull(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_positionRating,
                                                          const pcl::PointXYZ& check_pos,
                                                          const pcl::PointXYZ& support_point_1,
                                                          const pcl::PointXYZ& support_point_2,
                                                          const pcl::PointXYZ& support_point_3,
                                                          const bool iterative, // if this is true, points on one side of supp1, supp2 will not be considerred
                                                          pcl::PointCloud<pcl::PointXYZ>::Ptr ground_contact_points) // empty before call
{
    const pcl::PointXYZ support_plane_normal= crossProduct(pcl::PointXYZ(support_point_1.x-support_point_2.x,support_point_1.y-support_point_2.y,support_point_1.z-support_point_2.z),
                                                   pcl::PointXYZ(support_point_1.x-support_point_3.x,support_point_1.y-support_point_3.y,support_point_1.z-support_point_3.z));

    //Find ground contact points
    for (unsigned int i = 0; i < cloud_positionRating->size(); i++)
    {
        pcl::PointXYZI& p = cloud_positionRating->at(i);
        const float dist = planeDistance(pcl::PointXYZ(p.x,p.y,p.z),support_plane_normal,support_point_1);
        if(fabs(dist) < delta_for_contact)
        {
            if (iterative == false){
                ground_contact_points->push_back(pcl::PointXYZ(p.x,p.y,p.z));
            }
            else {
                pcl::PointXYZ pointXYZ = pcl::PointXYZ(p.x, p.y, p.z);
                // only add points on the right side of the supporting points 1 and 2
                if (sign(ccw(support_point_1, support_point_2, support_point_3)) == sign(ccw(support_point_1, support_point_2, pointXYZ))
                        || ccw(support_point_1, support_point_2, pointXYZ) == 0
                        //|| pointsEqual(support_point_1, pointXYZ, 0.0002)// should not be neccessary but due to floating numbers it is
                        //|| pointsEqual(support_point_2, pointXYZ, 0.0002)
                        )
                {
                    ground_contact_points->push_back(pointXYZ);
                }
            }
        }
        p.intensity=p.z;
    }

    // convex hull build here
    // first point is also last point
    std::vector<pcl::PointXYZ> convex_hull_points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(ground_contact_points);
    chull.reconstruct(*cloud_hull);
    for(unsigned int i = 0; i<cloud_hull->points.size(); ++i)
    {
        convex_hull_points.push_back(cloud_hull->at(i));
    }
    convex_hull_points.push_back(cloud_hull->at(0));    //for consistency with previous data structure

    if (distance_smoothing){
        //ROS_INFO("distance smoothing");
        // distance smoothing
        for (unsigned int i = 0; i < convex_hull_points.size() - 1; i = i){
            //         if (convex_hull_points.size() <= 4) break; // area needs to have 3 points
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
                    if (i == 0){ // do not delete first point
                        offset = 1;
                    }
                    convex_hull_points.erase(convex_hull_points.begin()+(i+offset));
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
                    //if (dist_p_i < dist_p_next){ // delete the second last point not the last
                    /*if (distanceXYZ(convex_hull_points.at(i), check_pos) < distanceXYZ(convex_hull_points.at(i+1), check_pos)){*/
                    // second last must be removed
                    convex_hull_points.erase(convex_hull_points.begin()+(i));
                    //}
                    /*else{
                        // last = first must be removed
                        convex_hull_points.erase(convex_hull_points.begin()+(i+1));
                        convex_hull_points.at(0) = convex_hull_points.at(i);
                        // ERROR: another point (first one must be pusheback to have enough points
                    }*/
                }
                break; // needed because loop is doing i = i // could also be i++
            }
        }

        //ROS_INFO("end distance smoothing");
    }

    //ROS_INFO("convex_hull_points.size = %i after distance smoothing, this should at least be 4 to have 3 points which is a valid triangle as area.", convex_hull_points.size());
    /* for (unsigned int i = 0; i < convex_hull_points.size(); i++){
        pcl::PointXYZ p = convex_hull_points.at(i);
        ROS_INFO("P%i x = %f, y = %f, z = %f", i, p.x, p.y, p.z);
    }*/

    // angle smoothing
    if (angle_smoothing){

        //ROS_INFO("angle smoothing");
        for (unsigned int i = 0; i < convex_hull_points.size() - 1; i = i){
            //        if (convex_hull_points.size() <= 4) break; // area needs to have 3 points
            if (i < convex_hull_points.size()-2){

                Eigen::Vector3f direction1 = subtractPointsEigen(convex_hull_points.at(i + 1),convex_hull_points.at(i));
                Eigen::Vector3f direction2 = subtractPointsEigen(convex_hull_points.at(i + 2),convex_hull_points.at(i+1));
                float angle = angleBetween(direction1, direction2);

                // delete the point if angle is very low
                if (angle < smooth_max_angle){
                    convex_hull_points.erase(convex_hull_points.begin()+(i+1));
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
                }
                break; // needed because loop is doing i = i // could also be i++
            }
        }
        //ROS_INFO("end angle smoothing");
    }


    //ROS_INFO("convex_hull_points.size = %i after angle smoothing, this should at least be 4 to have 3 points which is a valid triangle as area.", convex_hull_points.size());
    /*for (unsigned int i = 0; i < convex_hull_points.size(); i++){
        pcl::PointXYZ p = convex_hull_points.at(i);
        ROS_INFO("P%i x = %f, y = %f, z = %f", i, p.x, p.y, p.z);
    }*/

    return convex_hull_points;
}

void TerrainModel::computeRobotPosition(const pcl::PointXYZ support_point_1, const pcl::PointXYZ support_point_2, const pcl::PointXYZ support_point_3,
                                        const pcl::PointXYZ p0, const pcl::PointXYZ p1, const pcl::PointXYZ p2, const pcl::PointXYZ p3,
                                        const Eigen::Vector3f& offset_CM,
                                        pcl::PointXYZ& normal,
                                        pcl::PointXYZ& robot_point_0, pcl::PointXYZ& robot_point_1, pcl::PointXYZ& robot_point_2, pcl::PointXYZ& robot_point_3,
                                        pcl::PointXYZ& robot_point_mid, pcl::PointXYZ& robot_center_of_mass)
{

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

void TerrainModel::computeRobotCornerPoints(const pcl::PointXYZ& check_pos, const float orientation,
                                            pcl::PointXYZ& p0, pcl::PointXYZ& p1, pcl::PointXYZ& p2, pcl::PointXYZ& p3)
{
    p0 = pcl::PointXYZ(+ 0.5*robot_length, + 0.5*robot_width, 0);
    p0 = rotatePointZ(p0, orientation);
    p0 = addPointVector(p0, check_pos);

    p1 = pcl::PointXYZ(- 0.5*robot_length, + 0.5*robot_width, 0);
    p1 = rotatePointZ(p1, orientation);
    p1 = addPointVector(p1, check_pos);

    p2 = pcl::PointXYZ(- 0.5*robot_length, - 0.5*robot_width, 0);
    p2 = rotatePointZ(p2, orientation);
    p2 = addPointVector(p2, check_pos);

    p3 = pcl::PointXYZ(+ 0.5*robot_length, - 0.5*robot_width, 0);
    p3 = rotatePointZ(p3, orientation);
    p3 = addPointVector(p3, check_pos);
}

void TerrainModel::fillRobotPointcloud(const pcl::PointXYZ& p0, const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3, unsigned int& highest_Point_idx){

    robot_pcl.reset(new pcl::PointCloud<pcl::PointXYZI>());

    unsigned int n_counter=0;

    std::vector<pcl::PointXYZ> projected_robot_hull;
    projected_robot_hull.push_back(p0);
    projected_robot_hull.push_back(p1);
    projected_robot_hull.push_back(p2);
    projected_robot_hull.push_back(p3);

    bool hull_is_cw= (ccw(p0,p1,p2)<0);

    for (unsigned int i = 0; i < world_pcl_ptr->size(); i++)
    {
        const  pcl::PointXYZ& pp= world_pcl_ptr->at(i);

        bool c0 = hull_is_cw ? (ccw(p0,p1,pp)<0) : (ccw(p0,p1,pp)>0);
        bool c1 = hull_is_cw ? (ccw(p1,p2,pp)<0) : (ccw(p1,p2,pp)>0);
        bool c2 = hull_is_cw ? (ccw(p2,p3,pp)<0) : (ccw(p2,p3,pp)>0);
        bool c3 = hull_is_cw ? (ccw(p3,p0,pp)<0) : (ccw(p3,p0,pp)>0);

        if(c0 && c1 && c2 && c3) // point is in the area of the projected robot
        {
            pcl::PointXYZI p= pcl::PointXYZI();
            p.x=pp.x;
            p.y=pp.y;
            p.z=pp.z;
            p.intensity=0.0;
            robot_pcl->push_back(p);
            if(n_counter==0)
                highest_Point_idx = 0;
            else if(p.z>robot_pcl->at(highest_Point_idx).z)
                highest_Point_idx=n_counter;
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
bool TerrainModel::computePositionRating(const pcl::PointXYZ& flat_robot_center,
                                         const float orientation,
                                         pcl::PointXYZ& robot_point_center,
                                         pcl::PointXYZ& robot_point_0,
                                         pcl::PointXYZ& robot_point_1,
                                         pcl::PointXYZ& robot_point_2,
                                         pcl::PointXYZ& robot_point_3,
                                         float &position_rating, // after tipping over
                                         int &unstable_axis // of the first polygon
                                         , boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                                         int viewport_1, int viewport_2, int viewport_3, int viewport_4, bool use_visualization)
{    

#ifdef time_debug
    // times for debug / to see the speed of the algorithm
    double time_start_robotplcfind;
    double time_start_findSupPoints23;
    double time_start_computeHull;
    double time_start_positionRating;
    double time_duration_robotpclfind;
    double time_duration_findSupPoints23;
    double time_duration_computeHull;
    double time_duration_positionRating;
#endif

    position_rating = 0.0;
    unstable_axis = 0;

    // Points at the edges of the robot (flat ground)
    pcl::PointXYZ p0, p1, p2, p3; // Edge Points
    computeRobotCornerPoints(flat_robot_center, orientation, p0,p1,p2,p3);

#ifdef time_debug
    time_start_robotplcfind =ros::Time::now().toNSec();
#endif

    // fill the robot_pcl
    unsigned int highest_Point_idx;
    fillRobotPointcloud(p0, p1, p2, p3, highest_Point_idx);

    if(robot_pcl->size()==0) {
        ROS_WARN("[flor terrain classifier] No points for state estimation available. Planning into empty space?");
        return false;
    }
#ifdef time_debug
    time_duration_robotpclfind = (ros::Time::now().toNSec() - time_start_robotplcfind)/1000;
    ROS_INFO("time for robot cloud compute [mikrosec] = %i", (int)time_duration_robotpclfind);
#endif

    //ROS_INFO("cloud position rating size = %i", cloud_positionRating->size());

    pcl::PointXYZI &p_max_height= robot_pcl->at(highest_Point_idx);
    int support_point_1_idx = -1;
    float min_dist=-1.0;
    //choose highest point (+-delta) that is closest to CoM
    for (unsigned int i = 0; i < robot_pcl->size(); i++)
    {
        pcl::PointXYZI& p = robot_pcl->at(i);
        if(((p.z-p_max_height.z)<0.03)&&((p.z-p_max_height.z)>-0.03))
        {
            float dist=sqrt((p.x-flat_robot_center.x)*(p.x-flat_robot_center.x)+(p.y-flat_robot_center.y)*(p.y-flat_robot_center.y));
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
    std::vector<pcl::PointXYZ> convex_hull_points;

    int counter = 0;

    while (true){ // find SupportPoints ; runs multiple times if unstable axis occur

#ifdef time_debug
        time_start_findSupPoints23 =ros::Time::now().toNSec();
#endif
        // find supp P 2
        const pcl::PointXYZ tip_over_axis_point = support_point_1;
        pcl::PointXYZ tip_over_axis_vector = (crossProduct(pcl::PointXYZ(support_point_1.x-flat_robot_center.x,support_point_1.y-flat_robot_center.y,0),pcl::PointXYZ(0,0,1)));
        pcl::PointXYZ tip_over_direction;

        if (tip_over_axis_vector.x < MY_EPS && tip_over_axis_vector.y < MY_EPS && tip_over_axis_vector.z < MY_EPS) //todo think about appropriate solution
        {
            tip_over_axis_vector = pcl::PointXYZ(1.0, 0.0, 0.0); // this is random.
            tip_over_direction = pcl::PointXYZ(0.0, 1.0, 0.0); // 90° from vector
        }
        else
        {
            tip_over_direction = pcl::PointXYZ(flat_robot_center.x - support_point_1.x, flat_robot_center.y - support_point_1.y, 0);
        }

        bool point_evaluated = findSupportPoint(tip_over_axis_point,
                                                tip_over_axis_vector,
                                                *robot_pcl,
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
        if(ccw(support_point_1,support_point_2,flat_robot_center)>0)
            tip_over_direction_3 = crossProduct(tip_over_axis_vector_3, pcl::PointXYZ(0,0,-1));
        else
            tip_over_direction_3 = crossProduct(tip_over_axis_vector_3, pcl::PointXYZ(0,0,1));

        point_evaluated = findSupportPoint(tip_over_axis_point_3,
                                           tip_over_axis_vector_3,
                                           (*robot_pcl),
                                           tip_over_direction_3,
                                           support_point_3);
        if (!point_evaluated){
            ROS_WARN("[terrain_model::compute_position_rating] no support_point found 3 before tip over.");
            return false;
        }
#ifdef time_debug
        time_duration_findSupPoints23 = (ros::Time::now().toNSec() - time_start_findSupPoints23)/1000;
        ROS_INFO("time for finding support Points 2 and 3 compute [mikrosec] = %i", (int)time_duration_findSupPoints23);

        time_start_computeHull =ros::Time::now().toNSec();
#endif



        convex_hull_points = buildConvexHull(robot_pcl,
                                             flat_robot_center,
                                             support_point_1,
                                             support_point_2,
                                             support_point_3,
                                             false,
                                             ground_contact_points);
#ifdef time_debug
        time_duration_computeHull = (ros::Time::now().toNSec() - time_start_computeHull)/1000;
        ROS_INFO("time for computeHull [mikrosec] = %i", (int)time_duration_computeHull);
#endif


        if(use_visualization) // draw ground contact
        {
            if (draw_convex_hull_ground_points){
                for (unsigned int j = 0; j < ground_contact_points->size(); ++j)
                {
                    std::string name ="groundContactArea"+boost::lexical_cast<std::string>(j);
                    viewer->addSphere(ground_contact_points->at(j), 0.015,0.1 ,0.9,0.9, name, viewport_4);
                }
            }
        }

        // check if check_pos is in hull
        bool center_in_hull = true;


        //ROS_INFO("Elements in convex hull %i", convex_hull_points.size());
        for (unsigned int i = 0; i < convex_hull_points.size() -1; ++i){
            if (sign(ccw(convex_hull_points.at(i), convex_hull_points.at(i+1), flat_robot_center)) == -1){
                center_in_hull = false;
            }
        }


        if (center_in_hull == true || first_SP_around_mid == false){
            //ROS_INFO("check in hull is true");
            // The checkpos is in the convex hull. nothing needs to be changed
            break; // endwhile
        }

        counter++;

        // this can be expanded to an iterative process if iterations takes a higher value than 1.
        // Normally the check_pos is directly in the supporting polygon.
        // If not after the first iteration, it most likely is.
        int iterations = 15;
        if (counter > iterations){
            ROS_WARN("After %i iterations, the position to check is not in the supportiny polygon, computing rating continues.", iterations);
            break;
        }

        // checkpos is not in hull
        if (center_in_hull == false){
            //ROS_INFO("check in hull is false");
            // find closest point to checkpos
            float dist = distanceXY(convex_hull_points.at(0), flat_robot_center);
            int index_of_closest_point = 0;
            for (unsigned int i = 1; i < convex_hull_points.size(); ++i){
                float dist_to_check = distanceXY(convex_hull_points.at(i), flat_robot_center);
                if (dist_to_check < dist){
                    dist = dist_to_check;
                    index_of_closest_point = i;
                }
            }
            support_point_1 = convex_hull_points.at(index_of_closest_point);
            ground_contact_points->clear();
        }


    } // endwhile - the supporting polygon is around the check_pos now.

    // check if the position is instable due to an invalid steap angle
    float angle_of_area = angleToGround(support_point_1, support_point_2, support_point_3);
    if (angle_of_area > invalid_angle){
        ROS_INFO("the angle of the position is over %f degree, computePositionRating returned false", invalid_angle);
        return false;
    }

    //Compute points and normal of robot
    pcl::PointXYZ normal;
    pcl::PointXYZ center_of_mass;

    computeRobotPosition(support_point_1, support_point_2, support_point_3, p0, p1, p2, p3,
                         offset_CM,
                         normal,
                         robot_point_0, robot_point_1, robot_point_2, robot_point_3,
                         robot_point_center, center_of_mass);
if(use_visualization)
{
    // draw robot lines
    viewer->addLine(robot_point_0,robot_point_1,1.0,1.0,1.0,"f0");
    viewer->addLine(robot_point_1,robot_point_2,1.0,1.0,1.0,"f1");
    viewer->addLine(robot_point_2,robot_point_3,1.0,1.0,1.0,"f2");
    viewer->addLine(robot_point_3,robot_point_0,1.0,1.0,1.0,"f3");
}

    //Compute Force Angle Stability Metric
#ifdef time_debug
    time_start_positionRating =ros::Time::now().toNSec();
#endif
    std::vector<float> rating =computeForceAngleStabilityMetric(center_of_mass,convex_hull_points);
#ifdef time_debug
    time_duration_positionRating = (ros::Time::now().toNSec() - time_start_positionRating)/1000;
    ROS_INFO("time for rating [mikrosec] = %i", (int)time_duration_positionRating);
#endif



    if(use_visualization)
    {
    for (unsigned int i=0; i<rating.size();++i)
    {
        // rating between convex_hull_point (i and i+1)
        float c =rating.at(i);
        std::string name ="convex_hull_rating"+boost::lexical_cast<std::string>(i);
        const pcl::PointXYZ p1(convex_hull_points[i].x,convex_hull_points[i].y,convex_hull_points[i].z);
        const pcl::PointXYZ p2(convex_hull_points[i+1].x,convex_hull_points[i+1].y,convex_hull_points[i+1].z);

        //ROS_INFO("RATING r: %f p1: %f %f %f p2:%f %f %f", c,p1.x,p1.y,p1.z,p2.x,p2.y,p2.z);
        ROS_INFO("Rating num %i = %f", i, rating.at(i));

        if (draw_convex_hull_first_polygon){
            if(c<invalid_rating)
                viewer->addLine(p1,p2,1,0,c/invalid_rating,name);
            else
                viewer->addLine(p1,p2,0,(c-invalid_rating)/invalid_rating,(invalid_rating*2-c)/invalid_rating,name);

            // draw convex hull (points only)
            std::string namech ="convexhullstart"+boost::lexical_cast<std::string>(i);
            viewer->addSphere(convex_hull_points[i], 0.025,0,1,0, namech,2 /*viewport*/);
        }
    }

    viewer->addSphere(convex_hull_points[0], 0.025,1,0,0, "convexhullstart", 2);
    viewer->addSphere(flat_robot_center, 0.05,1,0,0, "checkPosition", viewport_4);
    viewer->addSphere(robot_point_center, 0.05,0,1,0, "proMidx", viewport_4);
    viewer->addSphere(center_of_mass, 0.05,0,0,1, "CM", viewport_4);


}

    for (unsigned int j = 0; j < rating.size(); j++){
        if (rating.at(j) < invalid_rating){
            unstable_axis += 1;
        }
    }

    // -------------------------------------------------------------------------//
    // -----------------------TIP OVER------------------------------------------//
    // -------------------------------------------------------------------------//

    if (tip_over){
        // iterative checking if still fine after flipping over invalid axis
        for (unsigned int i = 0; i < rating.size(); i++){
            if (check_each_axis || rating.at(i) < invalid_rating){ // instabil
                if(use_visualization)
                {
                ROS_INFO("Roboter kippt ueber Kante mit rating %f", rating.at(i));
}
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
                    //ROS_INFO("after tipping over: S1 and S2 lie at the aend of area, no S3 could be found");
                    continue; // supp1 and 2 lie at the end of the area, no suppP3 could be found, invalide state, break for-loop and return initial max rating value
                }

                float angle_of_area_it = angleToGround(support_point_1, support_point_2, support_point_3);
                if (angle_of_area_it > invalid_angle){
                    //ROS_INFO("after tipping over, angle would be too high: %f", angle_of_area_it);
                    continue;
                }


                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_positionRating_iterative(new pcl::PointCloud<pcl::PointXYZ>());

                std::vector<pcl::PointXYZ> convex_hull_points_it = buildConvexHull(robot_pcl,
                                                                                   flat_robot_center,
                                                                                   support_point_1,
                                                                                   support_point_2,
                                                                                   support_point_3,
                                                                                   true,
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
                for (unsigned int l = 0; l < iterator+1; l++){
                    // shift first of list to last of list
                    pcl::PointXYZ temp = convex_hull_points_it.at(0);
                    convex_hull_points_it.erase(convex_hull_points_it.begin()+0);
                    convex_hull_points_it.push_back(temp);
                }
                if(use_visualization)
                {
                // draw supporting points
                viewer->addSphere(support_point_1, 0.02,1,0,0, "sp1it_tippedOver"+boost::lexical_cast<std::string>(i), viewport_4);
                viewer->addSphere(support_point_2, 0.02,0,1,0, "sp2it_tippedOver"+boost::lexical_cast<std::string>(i), viewport_4);
                viewer->addSphere(support_point_3, 0.02,0,0,1, "sp3it_tippedOver"+boost::lexical_cast<std::string>(i), viewport_4);
                if (draw_convex_hull_iterative_ground_points){
                    for (unsigned int j = 0; j < cloud_positionRating_iterative->size(); ++j){
                        std::string name ="groundContactAreait"+boost::lexical_cast<std::string>(j)+"tippedOver"+boost::lexical_cast<std::string>(i);
                        viewer->addSphere(cloud_positionRating_iterative->at(j), 0.01,0,1,1, name, viewport_4);
                    }
                }

                // draw center
                if (i == 0){
                    viewer->addSphere(robot_point_mid_it, 0.04,0,1,0, "proMidxit", viewport_4);
                    viewer->addSphere(center_of_mass_it, 0.04,0,0,1, "CMit", viewport_4);
                    // draw normal
                    const pcl::PointXYZ p_uff(addPointVector(robot_point_center, normal));
                    viewer->addLine(robot_point_center,p_uff,1.0,1.0,1.0,"fnormalit",viewport_4);
                }
}


                //Compute Force Angle Stability Metric
                std::vector<float> rating_iterative =computeForceAngleStabilityMetric(center_of_mass_it,convex_hull_points_it);

                if(use_visualization)
                {
                for (unsigned int k=0; k<rating_iterative.size();++k)
                {

                    // rating between convex_hull_point (i and i+1)
                    float c =rating_iterative.at(k);

                    std::string name ="convex_hull_rating_it"+boost::lexical_cast<std::string>(k)+"tippedOver"+boost::lexical_cast<std::string>(i);
                    const pcl::PointXYZ p1(convex_hull_points_it[k].x,convex_hull_points_it[k].y,convex_hull_points_it[k].z);
                    const pcl::PointXYZ p2(convex_hull_points_it[k+1].x,convex_hull_points_it[k+1].y,convex_hull_points_it[k+1].z);


                    ROS_INFO("Rating %f", rating_iterative.at(k));
                    if (draw_convex_hull_iterative){

                        if(c<invalid_rating)
                            viewer->addLine(p1,p2,1,0,c/invalid_rating,name,viewport_4);
                        else
                            viewer->addLine(p1,p2,0,(c-invalid_rating)/invalid_rating,(invalid_rating*2-c)/invalid_rating,name,viewport_4);


                        // draw convex hull (points only)
                        std::string namech ="convexhullstart_it"+boost::lexical_cast<std::string>(k)+"tippedOver"+boost::lexical_cast<std::string>(i);
                        viewer->addSphere(convex_hull_points_it[k+1], 0.015,0,1,0, namech,2 /*viewport*/);
                    }

                }

                if (draw_convex_hull_iterative){
                    viewer->addSphere(convex_hull_points_it[0], 0.015,1,0,0, "convexhullstart_it_tippedOver"+boost::lexical_cast<std::string>(i), 2 /*viewport*/);
                }
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

    position_rating = *std::min_element(rating.begin(),rating.end());

    // draw groundpoints
    if(use_visualization)
    {
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(robot_pcl, "intensity");
    // viewer->addPointCloud<pcl::PointXYZI>(robot_pcl,intensity_distribution, "positionRating_cloud", viewport);
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "positionRating" + std::string("_edges"), viewport);
    ROS_INFO("CPR done: position_rating = %f, unstable_axis = %i", position_rating, unstable_axis);
}


#ifdef time_debug
    double time_together = time_duration_robotpclfind +
            time_duration_findSupPoints23 +
            time_duration_computeHull +
            time_duration_positionRating;
    ROS_INFO("time for evaluated time [mikrosec] %i", (int)time_together);
#endif

    return true;
}

}
