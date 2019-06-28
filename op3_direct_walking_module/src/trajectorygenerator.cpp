#include "trajectorygenerator.h"
#include <iostream>

namespace  robotis_op{

TrajectoryGenerator::TrajectoryGenerator()
{
  //pelvis_pose << 1.0, 0.0, 0.0, 0.0,
  //               0.0, 1.0, 0.0, 0.0,
  //               0.0, 0.0, 1.0, MAXHEIGHT,
  //               0.0, 0.0, 0.0, 1.0;
  //
  //rfoot_pose <<  1.0, 0.0, 0.0, 0.0,
  //               0.0, 1.0, 0.0, -YOFFSET,
  //               0.0, 0.0, 1.0, 0.0,
  //               0.0, 0.0, 0.0, 1.0;
  //
  //lfoot_pose <<  1.0, 0.0, 0.0, 0.0,
  //               0.0, 1.0, 0.0, YOFFSET,
  //               0.0, 0.0, 1.0, 0.0,
  //               0.0, 0.0, 0.0, 1.0;

}

TrajectoryGenerator::~TrajectoryGenerator()
{

}

void TrajectoryGenerator::initialize(Eigen::Matrix4d pel_pose, Eigen::Matrix4d rf_pose,
                                     Eigen::Matrix4d lf_pose)
{
  pelvis_pose = pel_pose;
  rfoot_pose  = rf_pose;
  lfoot_pose  = lf_pose;
}

Eigen::Matrix4d TrajectoryGenerator::getInverseTransform(Eigen::Matrix4d transf)
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

Eigen::Vector3d TrajectoryGenerator::getRPY(Eigen::Matrix3d rot_m)
{
  Eigen::Vector3d ea = rot_m.eulerAngles(0, 1, 2);

  return ea;

}

Eigen::Vector3d TrajectoryGenerator::getRPY(Eigen::Matrix4d transf_m)
{
  Eigen::Matrix3d r;
  r << transf_m.block(0,0,3,3);
  Eigen::Vector3d ea = r.eulerAngles(0, 1, 2);

  return ea;

}

Eigen::Matrix3d TrajectoryGenerator::rpyToRotM(Eigen::Vector3d rpy)
{
  Eigen::AngleAxisd rollAngle  (rpy(0), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle (rpy(1), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle   (rpy(2), Eigen::Vector3d::UnitZ());

  Eigen::Quaternion<double> q = rollAngle * pitchAngle *  yawAngle;

  Eigen::Matrix3d rotationMatrix = q.matrix();

  return rotationMatrix;
}

void TrajectoryGenerator::comTranslation(leg_type leg, Eigen::Matrix4d goal_pose)
{
  // CoM translation wrt Base frame
  if(leg == RIGHT){
    rleg_traj.push_back(pelvis_pose * rfoot_pose * this->getInverseTransform(goal_pose));
    //std::cout<<pelvis_pose * rfoot_pose * this->getInverseTransform(goal_pose)<<std::endl;
  }

  if(leg == LEFT){
    lleg_traj.push_back(pelvis_pose * lfoot_pose * this->getInverseTransform(goal_pose));
    //std::cout<<pelvis_pose * lfoot_pose * this->getInverseTransform(goal_pose)<<std::endl;
  }

}

void TrajectoryGenerator::footTranslation(leg_type leg, Eigen::Matrix4d goal_pose)
{
  // Foot translation wrt Base frame
  if(leg == RIGHT){
    rleg_traj.push_back(goal_pose);
    //std::cout<<pelvis_pose * rfoot_pose * this->getInverseTransform(goal_pose)<<std::endl;
  }

  if(leg == LEFT){
    lleg_traj.push_back(goal_pose);
    //std::cout<<pelvis_pose * lfoot_pose * this->getInverseTransform(goal_pose)<<std::endl;
  }

}

Eigen::Vector3d TrajectoryGenerator::getDeltaRPY(Eigen::Matrix4d init_pose, Eigen::Matrix4d goal_pose, unsigned int num_steps)
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

Eigen::Matrix4d TrajectoryGenerator::getDeltaTransf(Eigen::Matrix4d init_pose, Eigen::Matrix4d goal_pose,
                                                    unsigned int num_steps)
{ // Transformation matrix as a difference

  Eigen::Vector3d delta_rpy;
  delta_rpy = this->getDeltaRPY(init_pose, goal_pose, num_steps);

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

void TrajectoryGenerator::pevlisTranslation(double operating_rate, double duration, Eigen::Matrix4d goal_pose)
{
  unsigned int num_steps = (unsigned int) duration * operating_rate;

  Eigen::Matrix3d r_init;
  r_init = pelvis_pose.block(0,0,3,3);
  Eigen::Vector3d rpy_init;
  rpy_init = this->getRPY(r_init);

  Eigen::Vector3d delta_rpy;
  delta_rpy = this->getDeltaRPY(pelvis_pose, goal_pose, num_steps);

  Eigen::Matrix4d delta_transf;
  delta_transf = this->getDeltaTransf(pelvis_pose, goal_pose, num_steps);

  //std::cout<<delta_transf<<std::endl<<std::endl;

  Eigen::Vector3d cur_rpy;
  cur_rpy = rpy_init;

  Eigen::Matrix3d cur_rot;

  Eigen::Matrix4d cur_pose;
  cur_pose = pelvis_pose;

  this->comTranslation(this->RIGHT, cur_pose);
  this->comTranslation(this->LEFT, cur_pose);

  for(unsigned int i=0; i<num_steps; i++)
  {
    cur_rpy += delta_rpy;
    cur_rot = this->rpyToRotM(cur_rpy);

    cur_pose << cur_rot(0,0), cur_rot(0,1), cur_rot(0,2), (cur_pose(0,3) + delta_transf(0,3)),
                cur_rot(1,0), cur_rot(1,1), cur_rot(1,2), (cur_pose(1,3) + delta_transf(1,3)),
                cur_rot(2,0), cur_rot(2,1), cur_rot(2,2), (cur_pose(2,3) + delta_transf(2,3)),
                         0.0,          0.0,          0.0,                                 1.0;

    this->comTranslation(this->RIGHT, cur_pose);
    this->comTranslation(this->LEFT, cur_pose);

  }

  //pelvis_pose = goal_pose;

}

void TrajectoryGenerator::getTrajectory(leg_type leg, std::vector <Eigen::Matrix4d> &traj_vec)
{
  if(leg == RIGHT)
    traj_vec.assign(rleg_traj.begin(), rleg_traj.end());
  if(leg == LEFT)
    traj_vec.assign(lleg_traj.begin(), lleg_traj.end());

}




}
