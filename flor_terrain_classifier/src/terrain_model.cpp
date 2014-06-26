#include <flor_terrain_classifier/terrain_model.h>

namespace hector_terrain_model
{

TerrainModel::TerrainModel()
{}

TerrainModel::TerrainModel(pcl::PointCloud<pcl::PointXYZ> cloud)
{
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

/*--------------------------------------------------------------------------------------------*/
/* ---------------Helper Functions for Eigen Vector3 and PointXYZ ----------------------------*/
/*--------------------------------------------------------------------------------------------*/

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
    return acos(dotproductEigen(v1,v2)/
                (sqrt(dotproductEigen(v1,v1)) * sqrt(dotproductEigen(v2,v2))))
            * (180.0 / PI);
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

// 1 if positive, 0 if zero, -1 if negative
int sign(float a){
    if (a == 0) return 0;
    if (a > 0) return 1;
    return -1;
}

// checks if 2 vectors are similar; needed for deletion of convex_hull_ dge. TODO check reference / copy
bool check_similar_vector(pcl::PointXYZ support_point_1, pcl::PointXYZ support_point_2,
                          pcl::PointXYZ hull_p1, pcl::PointXYZ  hull_p2,
                          float similarity_distance, float similarity_angle){
    // check angle similarity
    float angle = angleBetween(subtractPointsEigen(support_point_1, support_point_2), subtractPointsEigen(hull_p1, hull_p2));
    if (angle > similarity_angle)
        return false;
    //check distance similarity evaluated by point distances
    float average_dist = (distanceXYZ(support_point_1, hull_p2) + distanceXYZ(support_point_2, hull_p2)) / 2.0;
    if (average_dist > similarity_distance)
        return false;
    return true;
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
// if tip_over_axis_point is returned, there could not be a point found.
pcl::PointXYZ TerrainModel::eval_point(const pcl::PointXYZ& tip_over_axis_point,
                                            const pcl::PointXYZ& tip_over_axis_vector,
                                            const pcl::PointCloud<pcl::PointXYZI>& pointcloud_robo,
                                            const pcl::PointXYZ& tip_over_direction)
{
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
    float min_angle=360.0;  // Hier den angle von 5 auf 360 geändert, da der winkel auch generell größer sein kann.

    int min_angle_idx = -1;
    const pcl::PointXYZ v1 = tip_over_direction;
    std::vector<float> angles;


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

       // ROS_INFO("is side %i %i", side, directionccw);
        if (side == directionccw){
           angles.push_back(angle);
           if (angle<min_angle)
            {
              min_angle=angle;
              min_angle_idx=i;
             // ROS_INFO("newminangle : %f ",angle);
             }

        }
    }

    // IF NO POINT COULD BE FOUND
    if (min_angle_idx == -1){
        return tip_over_axis_point;
    }

    const pcl::PointXYZ support_point=pcl::PointXYZ(pointcloud_robo.at(min_angle_idx).x,pointcloud_robo.at(min_angle_idx).y,pointcloud_robo.at(min_angle_idx).z);
    return support_point;
}

std::vector<pcl::PointXYZ> TerrainModel:: build_convex_hull(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_positionRating,
                                             const pcl::PointXYZ& check_pos,
                                             const pcl::PointXYZ& support_point_1,
                                             const pcl::PointXYZ& support_point_2,
                                             const pcl::PointXYZ& support_point_3,
                                             std::vector<unsigned int>& convex_hull_indices, // empty before call
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_positionRating2){ // empty before call

    bool smooth_hull = true;
    float smooth_max_angle = 10.0;


    const pcl::PointXYZ final_normal= crossProduct(pcl::PointXYZ(support_point_1.x-support_point_2.x,support_point_1.y-support_point_2.y,support_point_1.z-support_point_2.z),
                                                   pcl::PointXYZ(support_point_1.x-support_point_3.x,support_point_1.y-support_point_3.y,support_point_1.z-support_point_3.z));


    //Find ground contact points
    for (unsigned int i = 0; i < cloud_positionRating->size(); i++)
    {
        pcl::PointXYZI& p = cloud_positionRating->at(i);
        const float dist = planeDistance(pcl::PointXYZ(p.x,p.y,p.z),final_normal,support_point_1);
        if(dist<0.01 && dist>-0.01)
        {
            cloud_positionRating2->push_back(pcl::PointXYZ(p.x,p.y,p.z));
        }
        p.intensity=p.z;
    }


    //const pcl::PointXYZ pcs=planeProjection(check_pos,final_normal,support_point_1);
    //viewer.addSphere(pcs,0.04,1,0,1, "cp_pro", viewport);  ???

    // convex hull build here
    // first point is also last point
    convex_hull_comp(*cloud_positionRating2, convex_hull_indices);

    std::vector<pcl::PointXYZ> convex_hull_points;

    // add points to the list
    for(int i=0; i<(convex_hull_indices.size()); ++i)
    {
        const pcl::PointXYZ p1(cloud_positionRating2->at(convex_hull_indices[i]).x,cloud_positionRating2->at(convex_hull_indices[i]).y,cloud_positionRating2->at(convex_hull_indices[i]).z);
        convex_hull_points.push_back(p1);
    }

    if (smooth_hull == true){
    // make hull a little smoother
    ROS_INFO("smoothing");
    for (unsigned int i = 0; i < convex_hull_points.size() - 1; i = i){
        //ROS_INFO("%i ", i);
        if (i < convex_hull_points.size()-2){
            Eigen::Vector3f direction1 = subtractPointsEigen(convex_hull_points.at(i + 1),convex_hull_points.at(i));
            Eigen::Vector3f direction2 = subtractPointsEigen(convex_hull_points.at(i + 2),convex_hull_points.at(i+1));
            float angle = angleBetween(direction1, direction2);
           // ROS_INFO("%f angle", angle);
            if (angle < smooth_max_angle){
              //  ROS_INFO("in if");
                convex_hull_points.erase(convex_hull_points.begin()+(i+1));
                convex_hull_indices.erase(convex_hull_indices.begin()+(i+1));
            }
            else {
                ++i;
            }
        }
        else {
            //ROS_INFO("in else");
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
            break; // not needed

        }
    }
    ROS_INFO("end smoothing");
    }
    return convex_hull_points;
}

void TerrainModel::compute_robot_positions(const pcl::PointXYZ support_point_1, const pcl::PointXYZ support_point_2, const pcl::PointXYZ support_point_3,
                                                const float x_max, const float x_min, const float y_max, const float y_min,
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

    float z0=(-(x_max-support_point_2.x)*normal.x-(y_max-support_point_2.y)*normal.y)/normal.z +support_point_2.z;
    float z1=(-(x_min-support_point_2.x)*normal.x-(y_max-support_point_2.y)*normal.y)/normal.z +support_point_2.z;
    float z2=(-(x_max-support_point_2.x)*normal.x-(y_min-support_point_2.y)*normal.y)/normal.z +support_point_2.z;
    float z3=(-(x_min-support_point_2.x)*normal.x-(y_min-support_point_2.y)*normal.y)/normal.z +support_point_2.z;


    robot_point_0 = pcl::PointXYZ(x_max,y_max,z0);
    robot_point_1 = pcl::PointXYZ(x_min,y_max,z1);
    robot_point_2 = pcl::PointXYZ(x_max,y_min,z2);
    robot_point_3 = pcl::PointXYZ(x_min,y_min,z3);

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
    // -----------------------------------------------------------------------------------
    // bezg p_pro 1 / 0
    robot_center_of_mass = compute_center_of_mass(robot_point_0, robot_point_1, normal_vector, robot_point_mid, offset_CM);



}

pcl::PointXYZ TerrainModel::compute_center_of_mass(const pcl::PointXYZ &p1_left,
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

float TerrainModel::computePositionRating(const pcl::PointXYZ& check_pos,
                                              const float orientation)
{


    bool usetippingover = true;
    bool draw_convex_hull_first_polygon = false;
    bool draw_convex_hull_iterative = true;

    Eigen::Vector3f offset_CM = Eigen::Vector3f(0.10,0.0,0.3);

  //   lastRatedPosition=check_pos;

     float widthx=0.50;
     float lengthy=1.20;

     // Points under robot
     cloud_positionRating.reset(new pcl::PointCloud<pcl::PointXYZI>());
     unsigned int highest_Point_idx;
     unsigned int n_counter=0;

     //filter relevant points and find max
     const float x_max=check_pos.x+cos(orientation)*widthx*0.5-sin(orientation)*lengthy*0.5;
     const float x_min=check_pos.x-cos(orientation)*widthx*0.5+sin(orientation)*lengthy*0.5;
     const float y_max=check_pos.y+sin(orientation)*widthx*0.5+cos(orientation)*lengthy*0.5;
     const float y_min=check_pos.y-sin(orientation)*widthx*0.5-cos(orientation)*lengthy*0.5;
     const pcl::PointXYZ p0=pcl::PointXYZ(x_min,y_min,0);
     const pcl::PointXYZ p1=pcl::PointXYZ(x_max,y_min,0);
     const pcl::PointXYZ p2=pcl::PointXYZ(x_max,y_max,0);
     const pcl::PointXYZ p3=pcl::PointXYZ(x_min,y_max,0);


     bool hull_cpp= (ccw(p0,p1,p2)<0);
     if(!cloud_processed_Ptr)
     {
         ROS_ERROR("[Terrain Model] Empty pointcloud");
         return 0.f;
     }

     uint32_t cloudSize=cloud_processed.size();
     for (uint32_t i = 0; i < cloudSize2; i++)
     {
       const pcl::PointXYZ& pp= cloud_processed.at(i);

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
         ROS_ERROR("[Terrain Model] Robotposition out of pointcloud %f, %f, %f",check_pos.x,check_pos.y,check_pos.z);
         return 0.0f;
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_positionRating2(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<unsigned int> convex_hull_indices;
    std::vector<pcl::PointXYZ> convex_hull_points;

    int counter = 0;
    while (true){


        // find supp P 2
        const pcl::PointXYZ tip_over_axis_point = support_point_1;
        const pcl::PointXYZ tip_over_axis_vector = (crossProduct(pcl::PointXYZ(support_point_1.x-check_pos.x,support_point_1.y-check_pos.y,0),pcl::PointXYZ(0,0,1)));
        const pcl::PointXYZ tip_over_direction = pcl::PointXYZ(check_pos.x - support_point_1.x, check_pos.y - support_point_1.y, 0);

        support_point_2 = eval_point(tip_over_axis_point,
                                                         tip_over_axis_vector,
                                                         (*cloud_positionRating),
                                                         tip_over_direction);

       //find third point
       const pcl::PointXYZ tip_over_axis_point_3 = support_point_1;
       const pcl::PointXYZ tip_over_axis_vector_3= subtractPoints(support_point_2, support_point_1);
       pcl::PointXYZ tip_over_direction_3 ;
       if(ccw(support_point_1,support_point_2,check_pos)>0)
             tip_over_direction_3 = crossProduct(tip_over_axis_vector_3, pcl::PointXYZ(0,0,-1));
       else
             tip_over_direction_3 = crossProduct(tip_over_axis_vector_3, pcl::PointXYZ(0,0,1));

       support_point_3 = eval_point(tip_over_axis_point_3,
                                                         tip_over_axis_vector_3,
                                                         (*cloud_positionRating),
                                                         tip_over_direction_3);


       convex_hull_points = build_convex_hull(cloud_positionRating,
                                              check_pos,
                                              support_point_1,
                                              support_point_2,
                                              support_point_3,
                                              convex_hull_indices,
                                              cloud_positionRating2);

         // check if check_pos is in hull
         bool CenterInHull = true;

         for (unsigned int i = 0; i < convex_hull_points.size(); ++i){
             if (i < convex_hull_points.size() - 1){
                 if (sign(ccw(convex_hull_points.at(i), convex_hull_points.at(i+1), check_pos)) == -1){
                     CenterInHull = false;
                 }
             }
         } // end for



         if (CenterInHull == true){
             ROS_INFO("check in hull is true");
             // The checkpos is in the convex hull. nothing needs to be changed
             break; // endwhile
         }

         // find supppolygon, if check_pos not in supppolygon do again with another supp p 1
         counter++;
         ROS_INFO("%i iteration while loop", counter);


         if (counter > 1){
             ROS_INFO("ERROR -Supporting polygon after %i itereations not found // check_pos not in hull- ERROR", counter);
                     break;
         }

         // checkpos is not in hull
         if (CenterInHull == false){
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
             cloud_positionRating2->clear();
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

    compute_robot_positions(support_point_1, support_point_2, support_point_3, x_max, x_min, y_max, y_min,
                            offset_CM,
                            normal,
                            robot_point_0, robot_point_1, robot_point_2, robot_point_3,
                            robot_point_mid, center_of_mass_iterative);

    // render points
 /*  viewer.addSphere(support_point_1, 0.02,1,0,0, "sp1", viewport);
   viewer.addSphere(support_point_2, 0.02,0,1,0, "sp2", viewport);
   viewer.addSphere(support_point_3, 0.02,0,0,1, "sp3", viewport);
   for (unsigned int i = 0; i < cloud_positionRating2->size(); ++i){
       std::string name ="groundContactArea"+boost::lexical_cast<std::string>(i);
       viewer.addSphere(cloud_positionRating2->at(i), 0.01,0,1,1, name, viewport);
   }
    // draw robot lines
    viewer.addLine(robot_point_0,robot_point_1,1.0,1.0,1.0,"f0");
    viewer.addLine(robot_point_1,robot_point_3,1.0,1.0,1.0,"f1");
    viewer.addLine(robot_point_3,robot_point_2,1.0,1.0,1.0,"f2");
    viewer.addLine(robot_point_2,robot_point_0,1.0,1.0,1.0,"f3");
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
//         if(c<invalid_rating)
//             viewer.addLine(p1,p2,1,0,c/invalid_rating,name);
//         else
//             viewer.addLine(p1,p2,0,(c-invalid_rating)/invalid_rating,(invalid_rating*2-c)/invalid_rating,name);

         // draw convex hull (points only)
         std::string namech ="convexhullstart"+boost::lexical_cast<std::string>(i);
//         viewer.addSphere(convex_hull_points[i], 0.02,0,1,0, namech,2 /*viewport*/);
         }
     }
     if (draw_convex_hull_first_polygon){
//        viewer.addSphere(convex_hull_points[0], 0.02,1,0,0, "convexhullstart", 2 /*viewport*/);
    }

     if (usetippingover){
     // iterative checking if still fine after flipping over invalid axis
     for (unsigned int i = 0; i < rating.size(); i++){
         if (rating.at(i) < invalid_rating){ // instabil
             ROS_INFO("Roboter kippt ueber Kante mit rating %f", rating.at(i));
             support_point_1 = convex_hull_points.at(i);
             support_point_2 = convex_hull_points.at(i+1);

             const pcl::PointXYZ tip_over_axis_vector= subtractPoints(support_point_2, support_point_1);
             pcl::PointXYZ tip_over_direction = crossProduct(tip_over_axis_vector, pcl::PointXYZ(0,0,1));


             support_point_3 = eval_point(support_point_1,
                                          tip_over_axis_vector,
                                          (*cloud_positionRating),
                                          tip_over_direction);

             if (support_point_1.x == support_point_3.x &&
                     support_point_1.y == support_point_3.y &&
                     support_point_1.z == support_point_3.z){
                 break; // supp1 and 2 lie at the end of the area, no suppP3 could be found, invalide state, break for-loop and return initial max rating value
             }



             std::vector<unsigned int> convex_hull_indices_iterative;
             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_positionRating_iterative(new pcl::PointCloud<pcl::PointXYZ>());

             std::vector<pcl::PointXYZ> convex_hull_points_iterative = build_convex_hull(cloud_positionRating,
                                                    check_pos,
                                                    support_point_1,
                                                    support_point_2,
                                                    support_point_3,
                                                    convex_hull_indices_iterative,
                                                    cloud_positionRating_iterative);

             // TODO next 4 parameters may be slow

             float similarity_distance = 0.01; // meter
             float similarity_angle = 3.0; // degree
             float sim_dist_add = similarity_distance;
             float sim_angle_add = similarity_angle;

             bool one_edge_removed = false;
             while (one_edge_removed == false){ // one edge must be removed
                for (unsigned int k = 0; k < convex_hull_points_iterative.size()-1; k++){
                    if (check_similar_vector(support_point_1, support_point_2,
                                            convex_hull_points_iterative.at(k), convex_hull_points_iterative.at(k+1),
                                            similarity_distance, similarity_angle)){
                        // put in right order and delete last element so a gap is made.
                        convex_hull_points_iterative.erase(convex_hull_points_iterative.begin()+ convex_hull_points_iterative.size()-1);
                        convex_hull_indices_iterative.erase(convex_hull_indices_iterative.begin() + convex_hull_indices_iterative.size() -1);
                        for (unsigned int l = 0; l < convex_hull_points_iterative.size()- (k+2); l++){ //TODO looks righ, but why k+2 and not k+1;; loop is run size - (k+2) times.
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

             // draw supporting points
//            viewer.addSphere(support_point_1, 0.02,1,0,0, "sp1it", viewport);
//            viewer.addSphere(support_point_2, 0.02,0,1,0, "sp2it", viewport);
//            viewer.addSphere(support_point_3, 0.02,0,0,1, "sp3it", viewport);
            for (unsigned int i = 0; i < cloud_positionRating_iterative->size(); ++i){
                std::string name ="groundContactAreait"+boost::lexical_cast<std::string>(i);
//                viewer.addSphere(cloud_positionRating_iterative->at(i), 0.01,0,1,1, name, viewport);
            }
             // draw robot lines
//             viewer.addLine(robot_point_0,robot_point_1,1.0,1.0,1.0,"f0it");
//             viewer.addLine(robot_point_1,robot_point_3,1.0,1.0,1.0,"f1it");
//             viewer.addLine(robot_point_3,robot_point_2,1.0,1.0,1.0,"f2it");
//             viewer.addLine(robot_point_2,robot_point_0,1.0,1.0,1.0,"f3it");
             // draw center
//             viewer.addSphere(check_pos, 0.05,1,0,0, "checkPositionit", viewport);
//             viewer.addSphere(robot_point_mid, 0.05,0,1,0, "proMidxit", viewport);
//             viewer.addSphere(center_of_mass_iterative, 0.05,0,0,1, "CMit", viewport);
             // draw normal
             const pcl::PointXYZ p_uff(addPointVector(robot_point_mid, normal));
//             viewer.addLine(robot_point_mid,p_uff,1.0,1.0,1.0,"fnormalit");


             //Compute Force Angle Stability Metric
             std::vector<float> rating_iterative =computeForceAngleStabilityMetric(center_of_mass_iterative,convex_hull_points_iterative);
             for (unsigned int k=0; k<rating_iterative.size();++k)
             {

                 // rating between convex_hull_point (i and i+1)
                 float c =rating_iterative.at(k);

                 // TODO insert number of iteration in name
                 std::string name ="convex_hull_rating_it"+boost::lexical_cast<std::string>(convex_hull_indices_iterative[k]);
                 const pcl::PointXYZ p1(convex_hull_points_iterative[k].x,convex_hull_points_iterative[k].y,convex_hull_points_iterative[k].z);
                 const pcl::PointXYZ p2(convex_hull_points_iterative[k+1].x,convex_hull_points_iterative[k+1].y,convex_hull_points_iterative[k+1].z);

                 //ROS_INFO("RATING r: %f p1: %f %f %f p2:%f %f %f", c,p1.x,p1.y,p1.z,p2.x,p2.y,p2.z);
                 ROS_INFO("Rating %f", rating_iterative.at(k));


                 if (draw_convex_hull_iterative){

//                 if(c<invalid_rating)
//                     viewer.addLine(p1,p2,1,0,c/invalid_rating,name);
//                 else
//                     viewer.addLine(p1,p2,0,(c-invalid_rating)/invalid_rating,(invalid_rating*2-c)/invalid_rating,name);

                 // draw convex hull (points only)
                 // TODO insert number of iteration in name
                 std::string namech ="convexhullstart_it"+boost::lexical_cast<std::string>(k);
//                 viewer.addSphere(convex_hull_points_iterative[k+1], 0.02,0,1,0, namech,2 /*viewport*/);
                 }
             }
             if (draw_convex_hull_iterative){
//                viewer.addSphere(convex_hull_points_iterative[0], 0.02,1,0,0, "convexhullstart_it", 2 /*viewport*/);
            }



             float min_value_rating_iterative =*std::min_element(rating_iterative.begin(),rating_iterative.end());

             // wenn gekippt und es hier stabiler ist, übernehme den schlechtesen Wert der gekippten Position an
             // der entsprechenden Stelle
             if (min_value_rating_iterative > rating.at(i)){
                 rating.at(i) = min_value_rating_iterative;
             }

         }
     }
     } // usetippingover end

     float min_value_rating=*std::min_element(rating.begin(),rating.end());
     ROS_INFO("min_value_rating = %f", min_value_rating);

    return min_value_rating;
}


}
