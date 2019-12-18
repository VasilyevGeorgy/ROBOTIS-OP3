#include "../include/op3_direct_walking_module/frames.h"
#include <iostream>

namespace  robotis_op{

frames::frames()
{

}

frames::~frames()
{

}

void frames::initialize(Eigen::Matrix4d pel_pose, Eigen::Matrix4d rf_pose,
                                     Eigen::Matrix4d lf_pose)
{
  pelvis_pose = pel_pose;
  rfoot_pose  = rf_pose;
  lfoot_pose  = lf_pose;
}

Eigen::Matrix4d frames::getInverseTransform(Eigen::Matrix4d transf)
{
  Eigen::Matrix3d r_t;
  r_t = (transf.block(0,0,3,3)).transpose();
  Eigen::Vector3d pos;
  pos << transf(0,3), transf(1,3), transf(2,3);
  pos = -r_t * pos;

  Eigen::Matrix4d inv_transf;
  inv_transf << r_t(0,0), r_t(0,1), r_t(0,2), pos(0),
                r_t(1,0), r_t(1,1), r_t(1,2), pos(1),
                r_t(2,0), r_t(2,1), r_t(2,2), pos(2),
                0.0,      0.0,      0.0,      1.0;

  return inv_transf;

}

Eigen::Vector3d frames::getRPY(Eigen::Matrix3d rot_m)
{
  Eigen::Vector3d ea = rot_m.eulerAngles(0, 1, 2);

  return ea;

}

Eigen::Vector3d frames::getRPY(Eigen::Matrix4d transf_m)
{
  Eigen::Matrix3d r;
  r << transf_m.block(0,0,3,3);
  Eigen::Vector3d ea = r.eulerAngles(0, 1, 2);

  return ea;

}

Eigen::Matrix3d frames::rpyToRotM(Eigen::Vector3d rpy)
{
  Eigen::AngleAxisd rollAngle  (rpy(0), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle (rpy(1), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle   (rpy(2), Eigen::Vector3d::UnitZ());

  Eigen::Quaternion<double> q = rollAngle * pitchAngle *  yawAngle;

  Eigen::Matrix3d rotationMatrix = q.matrix();

  return rotationMatrix;
}

Eigen::Matrix4d frames::comTranslation(leg_type leg, Eigen::Matrix4d goal_pose)
{
  // CoM translation wrt Base frame
  if(leg == RIGHT){
    //std::cout<<pelvis_pose * rfoot_pose * this->getInverseTransform(goal_pose)<<std::endl;
    return pelvis_pose * rfoot_pose * this->getInverseTransform(goal_pose);
  }

  if(leg == LEFT){
    //std::cout<<pelvis_pose * lfoot_pose * this->getInverseTransform(goal_pose)<<std::endl;
    return pelvis_pose * lfoot_pose * this->getInverseTransform(goal_pose);
  }

}

Eigen::Matrix4d frames::footTranslation(Eigen::Matrix4d goal_pose)
{
  // Foot translation wrt Base frame
  //std::cout<<pelvis_pose * rfoot_pose * this->getInverseTransform(goal_pose)<<std::endl;
  return goal_pose;

}

Eigen::Vector3d frames::getDiffRPY(Eigen::Matrix4d init_pose, Eigen::Matrix4d goal_pose,
                                   unsigned int num_steps)
{
  // Get Roll Pitch Yaw from init_pose-matrix
  Eigen::Matrix3d r_init;
  r_init = init_pose.block(0,0,3,3);
  Eigen::Vector3d rpy_init;
  rpy_init = this->getRPY(r_init);

  // Get Roll Pitch Yaw from goal_pose-matrix
  Eigen::Matrix3d r_goal;
  r_goal = goal_pose.block(0,0,3,3);
  Eigen::Vector3d rpy_goal;
  rpy_goal = this->getRPY(r_goal);

  // Get differences for RPY
  Eigen::Vector3d delta_rpy;
  delta_rpy = (rpy_goal - rpy_init)/num_steps;

  return delta_rpy;

}

Eigen::Vector3d frames::getDiffRPY(Eigen::Matrix3d init_orientation, Eigen::Matrix3d goal_orientation,
                                   unsigned int num_steps)
{
  Eigen::Vector3d rpy_init;
  rpy_init = this->getRPY(init_orientation);

  Eigen::Vector3d rpy_goal;
  rpy_goal = this->getRPY(goal_orientation);

  // Get differences for RPY
  Eigen::Vector3d delta_rpy;
  delta_rpy = (rpy_goal - rpy_init)/num_steps;

  return delta_rpy;
}


Eigen::Matrix4d frames::getDiffTransf(Eigen::Matrix4d init_pose, Eigen::Matrix4d goal_pose,
                                                    unsigned int num_steps)
{ // Transformation matrix as a difference

  Eigen::Vector3d delta_rpy;
  delta_rpy = this->getDiffRPY(init_pose, goal_pose, num_steps);

  Eigen::Matrix3d delta_rot;
  delta_rot = this->rpyToRotM(delta_rpy);

  // Differences for translation vector
  Eigen::Vector3d delta_vec;
  delta_vec << (goal_pose(0,3) - init_pose(0,3))/num_steps,
               (goal_pose(1,3) - init_pose(1,3))/num_steps,
               (goal_pose(2,3) - init_pose(2,3))/num_steps;

  // Transformation matrix of differences
  Eigen::Matrix4d delta_transf;
  delta_transf << delta_rot(0,0), delta_rot(0,1), delta_rot(0,2), delta_vec(0),
                  delta_rot(1,0), delta_rot(1,1), delta_rot(1,2), delta_vec(1),
                  delta_rot(2,0), delta_rot(2,1), delta_rot(2,2), delta_vec(2),
                             0.0,            0.0,            0.0,          1.0;

  return delta_transf;

}

}
