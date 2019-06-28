#include "op3_direct_walking.h"

namespace  robotis_op{

op3_direct_walking::op3_direct_walking()
{

}

op3_direct_walking::~op3_direct_walking()
{

}

void op3_direct_walking::initialization(Eigen::Matrix4d pel_pos, Eigen::Matrix4d rf_pos, Eigen::Matrix4d lf_pos)
{
  pelvis_pose = pel_pos;
  rfoot_pose  = rf_pos;
  lfoot_pose  = lf_pos;
}

void op3_direct_walking::pevlisTranslation(double operating_rate, double duration, Eigen::Matrix4d goal_pose)
{
  unsigned int num_steps = (unsigned int) duration * operating_rate;

  //rfoot_pose << 1.0, 0.0, 0.0, 0.0,
  //              0.0, 1.0, 0.0,-YOFFSET,
  //              0.0, 0.0, 1.0, 0.0,
  //              0.0, 0.0, 0.0, 1.0;
  //
  //lfoot_pose << 1.0, 0.0, 0.0, 0.0,
  //              0.0, 1.0, 0.0, YOFFSET,
  //              0.0, 0.0, 1.0, 0.0,
  //              0.0, 0.0, 0.0, 1.0;

  robotis_op::TrajectoryGenerator *tr_gen = new robotis_op::TrajectoryGenerator();
  tr_gen->initialize(pelvis_pose, rfoot_pose, lfoot_pose);

  Eigen::Matrix3d r_init;
  r_init = pelvis_pose.block(0,0,3,3);
  Eigen::Vector3d rpy_init;
  rpy_init = tr_gen->getRPY(r_init);

  Eigen::Vector3d delta_rpy;
  delta_rpy = tr_gen->getDiffRPY(pelvis_pose, goal_pose, num_steps);
  Eigen::Matrix4d delta_transf;
  delta_transf = tr_gen->getDiffTransf(pelvis_pose, goal_pose, num_steps);

  //std::cout<<delta_transf<<std::endl<<std::endl;

  Eigen::Vector3d cur_rpy;
  cur_rpy = rpy_init;
  Eigen::Matrix3d cur_rot;
  Eigen::Matrix4d cur_pose;
  cur_pose = pelvis_pose;

  // add push_back to traj_vector!!!

  tr_gen->comTranslation(tr_gen->RIGHT, cur_pose);
  tr_gen->comTranslation(tr_gen->LEFT, cur_pose);

  for(unsigned int i=0; i<num_steps; i++)
  {
    cur_rpy += delta_rpy;
    cur_rot = tr_gen->rpyToRotM(cur_rpy);

    cur_pose << cur_rot(0,0), cur_rot(0,1), cur_rot(0,2), (cur_pose(0,3) + delta_transf(0,3)),
                cur_rot(1,0), cur_rot(1,1), cur_rot(1,2), (cur_pose(1,3) + delta_transf(1,3)),
                cur_rot(2,0), cur_rot(2,1), cur_rot(2,2), (cur_pose(2,3) + delta_transf(2,3)),
                         0.0,          0.0,          0.0,                                 1.0;

    tr_gen->comTranslation(tr_gen->RIGHT, cur_pose);
    tr_gen->comTranslation(tr_gen->LEFT, cur_pose);

  }

  //pelvis_pose = goal_pose;

}

}
