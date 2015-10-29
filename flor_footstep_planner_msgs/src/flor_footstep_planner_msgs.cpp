#include <flor_footstep_planner_msgs/flor_footstep_planner_msgs.h>

namespace flor_footstep_planner_msgs
{
void quaternionToNormal(const geometry_msgs::Quaternion &q, geometry_msgs::Vector3 &n)
{
  tf::Quaternion q_tf;
  tf::quaternionMsgToTF(q, q_tf);

  double r, p, y;
  tf::Matrix3x3(q_tf).getRPY(r, p, y);

  RPYToNormal(r, p, y, n);
}

void normalToQuaternion(double yaw, const geometry_msgs::Vector3 &n, geometry_msgs::Quaternion &q)
{
  double r, p;
  normalToRP(yaw, n, r, p);
  q = tf::createQuaternionMsgFromRollPitchYaw(r, p, yaw);
}

void RPYToNormal(double r, double p, double y, geometry_msgs::Vector3 &n)
{
  double sin_roll = sin(r);
  double sin_pitch = sin(p);
  double sin_yaw = sin(y);
  double cos_yaw = cos(y);

  // rotate around z axis
  n.x = cos_yaw*sin_pitch + sin_yaw*sin_roll;
  n.y = sin_yaw*sin_pitch - cos_yaw*sin_roll;
  n.z = sqrt(1.0 - n.x*n.x + n.y*n.y);
}

void normalToRP(double yaw, const geometry_msgs::Vector3 &n, double &r, double &p)
{
  // inverse rotation around z axis
  double sin_yaw = sin(-yaw);
  double cos_yaw = cos(-yaw);

  r = -asin(sin_yaw*n.x + cos_yaw*n.y);
  p = asin(cos_yaw*n.x - sin_yaw*n.y);
}
}
