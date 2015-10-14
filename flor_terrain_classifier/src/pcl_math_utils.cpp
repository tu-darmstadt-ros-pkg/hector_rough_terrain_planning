#include <flor_terrain_classifier/terrain_model.h>

//#define time_debug;
#define MY_EPS 1e-6
namespace hector_terrain_model
{

// normal adding, subtracting points

bool  pointsEqual(const pcl::PointXYZ& p1,const  pcl::PointXYZ& p2){
    if (fabs(p1.x - p2.x) > MY_EPS)
        return false;
    if (fabs(p1.y - p2.y) > MY_EPS)
        return false;
    if (fabs(p1.z - p2.z) > MY_EPS)
        return false;
    return true;
}

bool  pointsEqualxy(const pcl::PointXYZ& p1,const  pcl::PointXYZ& p2){
    if (fabs(p1.x - p2.x) > MY_EPS)
        return false;
    if (fabs(p1.y - p2.y) > MY_EPS)
        return false;
    return true;
}

bool  pointsEqualxy(const pcl::PointXYZI& p1,const  pcl::PointXYZI& p2){
    if (fabs(p1.x - p2.x) > MY_EPS)
        return false;
    if (fabs(p1.y - p2.y) > MY_EPS)
        return false;
    return true;
}

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
pcl::PointXYZ rotatePointZ(pcl::PointXYZ p, float degree /*radiants*/){
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
    if (acosvalue > 1-MY_EPS){
        return 0.0;
    }
    if (acosvalue < -(1-MY_EPS)){
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

float planeDistance(const pcl::PointXYZ& testpoint, const pcl::PointXYZ& plane_n, const pcl::PointXYZ& plane_p)
{
    float d=dotProduct(pcl::PointXYZ(testpoint.x-plane_p.x, testpoint.y-plane_p.y, testpoint.z-plane_p.z), plane_n) /
            sqrt(plane_n.x*plane_n.x+plane_n.y*plane_n.y+plane_n.z*plane_n.z);
    return fabs(d);
}

pcl::PointXYZ planeProjection(const pcl::PointXYZ& projection_p, const pcl::PointXYZ& plane_n, const pcl::PointXYZ& plane_p)
{


    Eigen::Vector3f x = Eigen::Vector3f(projection_p.x,projection_p.y,projection_p.z);
    Eigen::Vector3f n = Eigen::Vector3f(plane_n.x,plane_n.y,plane_n.z);
    Eigen::Vector3f r = Eigen::Vector3f(plane_p.x,plane_p.y,plane_p.z);
    Eigen::Vector3f res = x - ((x-r).dot(n)/n.dot(n))*n;
    pcl::PointXYZ ret = pcl::PointXYZ(res[0],res[1],res[2]);
    return ret;
}



bool atPlaneTest(const pcl::PointXYZ& testpoint, const pcl::PointXYZ& plane_n, const pcl::PointXYZ& plane_p)
{
    return (abs(planeDistance(testpoint,plane_n,plane_p))<=MY_EPS);
}

}
