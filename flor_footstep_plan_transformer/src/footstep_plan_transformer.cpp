#include <flor_footstep_plan_transformer/footstep_plan_transformer.h>

namespace flor_navigation
{
FootstepPlanTransformer::FootstepPlanTransformer()
{
  //Set transform to identity at beginning
  identity_transform_  = true;
  transform_world_to_bdi.setIdentity();
  transform_world_to_bdi.stamp_ = ros::Time();
  transform_world_to_bdi.frame_id_ = "/world";
  transform_world_to_bdi.child_frame_id_ = "/bdi_world";

  transform_bdi_to_world = tf::StampedTransform(transform_world_to_bdi.inverse(), ros::Time(), "/bdi_world", "/world");
}

void FootstepPlanTransformer::setTransform(const geometry_msgs::PoseStamped& transform_in)
{
  transform_world_to_bdi.setOrigin(tf::Vector3(transform_in.pose.position.x, transform_in.pose.position.y, transform_in.pose.position.z));
  transform_world_to_bdi.setRotation(tf::Quaternion(transform_in.pose.orientation.x,
                                                    transform_in.pose.orientation.y,
                                                    transform_in.pose.orientation.z,
                                                    transform_in.pose.orientation.w));

  // Flag if this is sufficiently close to identity transform
  identity_transform_ = ((tf::Vector3(transform_in.pose.position.x, transform_in.pose.position.y, transform_in.pose.position.z).length() < 1e-6) &&
                            (fabs(transform_in.pose.orientation.w) > 0.99999));

  transform_world_to_bdi.stamp_ = transform_in.header.stamp;
  transform_world_to_bdi.frame_id_ = transform_in.header.frame_id;
  transform_world_to_bdi.child_frame_id_ = "/bdi_world";

  transform_bdi_to_world = tf::StampedTransform(transform_world_to_bdi.inverse(), transform_in.header.stamp, "/bdi_world", transform_in.header.frame_id);
}

void FootstepPlanTransformer::transformYaw(float& yaw, const tf::StampedTransform &transf) const
{
  // generate point from yaw
  geometry_msgs::Point p;
  p.x = tfCos(yaw);
  p.y = tfSin(yaw);

  // transform point
  transform(p, transf);

  // get yaw of transformed point
  yaw = tfAtan2(p.y, p.x); // atan2 uses (y,x) arguments for tan = y/x
}

void FootstepPlanTransformer::transform(flor_footstep_planner_msgs::FootstepPlan& data, const tf::StampedTransform& transf) const
{
  transform(data.header, transf);
  transform(data.step_plan, transf);
}

void FootstepPlanTransformer::transform(flor_footstep_planner_msgs::StepTarget& step, const tf::StampedTransform& transf) const
{
  transform(step.header, transf);
  //transform(step.foot, transf);
}

void FootstepPlanTransformer::transform(flor_footstep_planner_msgs::FeetPoses& feet_poses, const tf::StampedTransform& transf) const
{
  transform(feet_poses.header, transf);
  //transform(feet_poses.left, transf);
  //transform(feet_poses.right, transf);
}

void FootstepPlanTransformer::transform(std_msgs::Header& header, const tf::StampedTransform& transf) const
{
  header.frame_id = transf.child_frame_id_;
}

void FootstepPlanTransformer::transform(geometry_msgs::Vector3& vec, const tf::StampedTransform& transf) const
{
  geometry_msgs::Point point;
  flor_footstep_planner_msgs::copyPosition(vec, point);
  transform(point, transf);
  flor_footstep_planner_msgs::copyPosition(point, vec);
}

void FootstepPlanTransformer::transform(geometry_msgs::Point& point, const tf::StampedTransform& transf) const
{
  tf::Point point_tf;
  tf::pointMsgToTF(point, point_tf);
  point_tf = transf * point_tf;
  tf::pointTFToMsg(point_tf, point);
}

void FootstepPlanTransformer::transform(geometry_msgs::Pose& pose, const tf::StampedTransform& transf) const
{
  tf::Pose pose_tf;
  tf::poseMsgToTF(pose, pose_tf);
  pose_tf = transf * pose_tf;
  tf::poseTFToMsg(pose_tf, pose);
}



void addOriginShift(geometry_msgs::Point& point, double yaw, const geometry_msgs::Vector3& shift)
{
  geometry_msgs::Pose pose;
  pose.position = point;
  pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  addOriginShift(pose, shift);
  point = pose.position;
}

void addOriginShift(geometry_msgs::Pose& pose, const geometry_msgs::Vector3& shift)
{
  tf::Pose transform;
  tf::poseMsgToTF(pose, transform);
  tf::Vector3 shift_world;
  tf::vector3MsgToTF(shift, shift_world);
  shift_world = transform.getBasis() * shift_world;

  pose.position.x += shift_world.getX();
  pose.position.y += shift_world.getY();
  pose.position.z += shift_world.getZ();
}

void addOriginShift(flor_footstep_planner_msgs::StepTarget& step, const geometry_msgs::Vector3& shift)
{
  geometry_msgs::Vector3 shift_foot = shift;
  //if (step.foot_index == flor_atlas_msgs::AtlasBehaviorFootData::FOOT_LEFT)
  //  shift_foot.y = -shift_foot.y;

  //addOriginShift(step.foot, shift_foot);
}

void addOriginShift(flor_footstep_planner_msgs::FeetPoses& feet, const geometry_msgs::Vector3& shift)
{
  geometry_msgs::Vector3 shift_left = shift;
  shift_left.y = -shift_left.y;

  //addOriginShift(feet.left, shift_left);
  //addOriginShift(feet.right, shift);
}

void removeOriginShift(geometry_msgs::Point& point, double yaw, const geometry_msgs::Vector3& shift)
{
  geometry_msgs::Pose pose;
  pose.position = point;
  pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  removeOriginShift(pose, shift);
  point = pose.position;
}

void removeOriginShift(geometry_msgs::Pose& pose, const geometry_msgs::Vector3& shift)
{
  tf::Pose transform;
  tf::poseMsgToTF(pose, transform);
  tf::Vector3 shift_world;
  tf::vector3MsgToTF(shift, shift_world);
  shift_world = transform.getBasis() * shift_world;

  pose.position.x -= shift_world.getX();
  pose.position.y -= shift_world.getY();
  pose.position.z -= shift_world.getZ();
}

void removeOriginShift(flor_footstep_planner_msgs::StepTarget& step, const geometry_msgs::Vector3& shift)
{
  geometry_msgs::Vector3 shift_foot = shift;
  //if (step.foot_index == flor_atlas_msgs::AtlasBehaviorFootData::FOOT_LEFT)
  //  shift_foot.y = -shift_foot.y;

  //removeOriginShift(step.foot, shift_foot);
}

void removeOriginShift(flor_footstep_planner_msgs::FeetPoses& feet, const geometry_msgs::Vector3& shift)
{
  geometry_msgs::Vector3 shift_left = shift;
  shift_left.y = -shift_left.y;

  //removeOriginShift(feet.left, shift_left);
  //removeOriginShift(feet.right, shift);
}
}
