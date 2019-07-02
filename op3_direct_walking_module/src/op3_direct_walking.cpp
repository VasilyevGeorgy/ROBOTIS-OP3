#include "../include/op3_direct_walking_module/op3_direct_walking.h"

namespace  robotis_op{

op3_direct_walking::op3_direct_walking()
{

  rleg_cur_jnt_pos.resize(JOINT_NUM);
  lleg_cur_jnt_pos.resize(JOINT_NUM);

  rleg_cur_jnt_pos << 0.0, 0.0, 0.01, -0.02, 0.0, 0.01;
  lleg_cur_jnt_pos << 0.0, 0.01, -0.01, 0.02, 0.01, 0.01;

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

void op3_direct_walking::getDelay(double operating_rate, double duration,
                                  std::vector<Eigen::VectorXd> &rleg_ang,
                                  std::vector<Eigen::VectorXd> &lleg_ang)
{

  unsigned int num_steps = (unsigned int) duration * operating_rate;

  for(unsigned int i=0; i<num_steps; i++)
  {
    rleg_ang.push_back(rleg_cur_jnt_pos);
    lleg_ang.push_back(lleg_cur_jnt_pos);
  }

}

void op3_direct_walking::movePelvis(double operating_rate, double duration, Eigen::Matrix4d goal_pose,
                                    std::vector<Eigen::VectorXd> &rleg_ang,
                                    std::vector<Eigen::VectorXd> &lleg_ang)
{
  unsigned int num_steps = (unsigned int) duration * operating_rate;

  robotis_op::frames *frames = new robotis_op::frames();
  robotis_op::KinSolver *kin_solver = new robotis_op::KinSolver();

  frames->initialize(pelvis_pose, rfoot_pose, lfoot_pose);
  kin_solver->initSolver(pelvis_pose);

  Eigen::Matrix3d r_init;
  r_init = pelvis_pose.block(0,0,3,3);
  Eigen::Vector3d rpy_init;
  rpy_init = frames->getRPY(r_init);

  Eigen::Vector3d delta_rpy;
  delta_rpy = frames->getDiffRPY(pelvis_pose, goal_pose, num_steps);
  Eigen::Matrix4d delta_transf;
  delta_transf = frames->getDiffTransf(pelvis_pose, goal_pose, num_steps);

  //std::cout<<delta_transf<<std::endl<<std::endl;

  // Current pose & rotation matrices initialization
  Eigen::Vector3d cur_rpy;
  cur_rpy = rpy_init;
  Eigen::Matrix3d cur_rot;
  Eigen::Matrix4d cur_pose;
  cur_pose = pelvis_pose;

  // IK solver variables
  Eigen::VectorXd r_jnt_pos(JOINT_NUM);
  Eigen::VectorXd l_jnt_pos(JOINT_NUM);

  rleg_traj.push_back(frames->comTranslation(frames->RIGHT, cur_pose));
  lleg_traj.push_back(frames->comTranslation(frames->LEFT, cur_pose));

  kin_solver->solveIK(kin_solver->RIGHT, rleg_cur_jnt_pos, rleg_traj.back(), r_jnt_pos);
  rleg_ang.push_back(r_jnt_pos);
  rleg_cur_jnt_pos = r_jnt_pos;

  kin_solver->solveIK(kin_solver->LEFT, lleg_cur_jnt_pos, lleg_traj.back(), l_jnt_pos);
  lleg_ang.push_back(l_jnt_pos);
  lleg_cur_jnt_pos = l_jnt_pos;

  for(unsigned int i=0; i<num_steps; i++)
  {
    cur_rpy += delta_rpy;
    cur_rot = frames->rpyToRotM(cur_rpy);

    cur_pose << cur_rot(0,0), cur_rot(0,1), cur_rot(0,2), (cur_pose(0,3) + delta_transf(0,3)),
                cur_rot(1,0), cur_rot(1,1), cur_rot(1,2), (cur_pose(1,3) + delta_transf(1,3)),
                cur_rot(2,0), cur_rot(2,1), cur_rot(2,2), (cur_pose(2,3) + delta_transf(2,3)),
                         0.0,          0.0,          0.0,                                 1.0;

    rleg_traj.push_back(frames->comTranslation(frames->RIGHT, cur_pose));
    lleg_traj.push_back(frames->comTranslation(frames->LEFT, cur_pose));

    kin_solver->solveIK(kin_solver->RIGHT, rleg_cur_jnt_pos, rleg_traj.back(), r_jnt_pos);
    rleg_ang.push_back(r_jnt_pos);
    rleg_cur_jnt_pos = r_jnt_pos;

    kin_solver->solveIK(kin_solver->LEFT, lleg_cur_jnt_pos, lleg_traj.back(), l_jnt_pos);
    lleg_ang.push_back(l_jnt_pos);
    lleg_cur_jnt_pos = l_jnt_pos;

  }

  //Rewrite positiom wrt Base frame
  rfoot_pose(0,3) = -goal_pose(0,3);
  lfoot_pose(0,3) = -goal_pose(0,3);

  goal_pose(0,3) = 0.0;
  pelvis_pose = goal_pose;

  kin_solver->deleteSolver();
  delete kin_solver;
  delete frames;

}

void op3_direct_walking::moveFoot(leg_type leg, double operating_rate, double duration, Eigen::Matrix4d goal_pose,
                                  std::vector<Eigen::VectorXd> &rleg_ang, std::vector<Eigen::VectorXd> &lleg_ang)
{
  unsigned int num_steps = (unsigned int) duration * operating_rate;

  robotis_op::frames *frames = new robotis_op::frames();
  robotis_op::KinSolver *kin_solver = new robotis_op::KinSolver();

  frames->initialize(pelvis_pose, rfoot_pose, lfoot_pose);
  kin_solver->initSolver(pelvis_pose);

  Eigen::Matrix3d r_init;
  Eigen::Vector3d rpy_init;
  // Posititon and rotation differences
  Eigen::Vector3d delta_rpy;
  Eigen::Matrix4d delta_transf;

  if(leg == RIGHT)
  {
    r_init = rfoot_pose.block(0,0,3,3);
    rpy_init = frames->getRPY(r_init);
    delta_rpy = frames->getDiffRPY(rfoot_pose, goal_pose, num_steps);
    delta_transf = frames->getDiffTransf(rfoot_pose, goal_pose, num_steps);
  }
  else
  {
    if(leg == LEFT){
      r_init = lfoot_pose.block(0,0,3,3);
      rpy_init = frames->getRPY(r_init);
      delta_rpy = frames->getDiffRPY(lfoot_pose, goal_pose, num_steps);
      delta_transf = frames->getDiffTransf(lfoot_pose, goal_pose, num_steps);
    }
    else
    {
      return;
    }
  }

  // Current pose & rotation matrices initialization
  Eigen::Vector3d cur_rpy;
  cur_rpy = rpy_init;
  Eigen::Matrix3d cur_rot;
  Eigen::Matrix4d cur_pose;

  if(leg == RIGHT)
    cur_pose = rfoot_pose;
  if(leg == LEFT)
    cur_pose = lfoot_pose;

  // IK solver variables
  Eigen::VectorXd des_jnt_pos(JOINT_NUM);

  if(leg == RIGHT)
  {
    rleg_traj.push_back(frames->footTranslation(cur_pose));
    kin_solver->solveIK(kin_solver->RIGHT, rleg_cur_jnt_pos, rleg_traj.back(), des_jnt_pos);
    rleg_ang.push_back(des_jnt_pos);
    rleg_cur_jnt_pos = des_jnt_pos;

    lleg_ang.push_back(lleg_cur_jnt_pos);
  }
  if(leg == LEFT)
  {
    lleg_traj.push_back(frames->footTranslation(cur_pose));
    kin_solver->solveIK(kin_solver->LEFT, lleg_cur_jnt_pos, lleg_traj.back(), des_jnt_pos);
    lleg_ang.push_back(des_jnt_pos);
    lleg_cur_jnt_pos = des_jnt_pos;

    rleg_ang.push_back(rleg_cur_jnt_pos);
  }

  for(unsigned int i=0; i<num_steps; i++)
  {
    cur_rpy += delta_rpy;
    cur_rot = frames->rpyToRotM(cur_rpy);

    cur_pose << cur_rot(0,0), cur_rot(0,1), cur_rot(0,2), (cur_pose(0,3) + delta_transf(0,3)),
                cur_rot(1,0), cur_rot(1,1), cur_rot(1,2), (cur_pose(1,3) + delta_transf(1,3)),
                cur_rot(2,0), cur_rot(2,1), cur_rot(2,2), (cur_pose(2,3) + delta_transf(2,3)),
                         0.0,          0.0,          0.0,                                 1.0;

    //std::cout<<frames->footTranslation(cur_pose)<<std::endl;

    if(leg == RIGHT)
    {
      rleg_traj.push_back(frames->footTranslation(cur_pose));
      kin_solver->solveIK(kin_solver->RIGHT, rleg_cur_jnt_pos, rleg_traj.back(), des_jnt_pos);
      rleg_ang.push_back(des_jnt_pos);
      rleg_cur_jnt_pos = des_jnt_pos;

      lleg_ang.push_back(lleg_cur_jnt_pos);
    }
    if(leg == LEFT)
    {
      lleg_traj.push_back(frames->footTranslation(cur_pose));
      kin_solver->solveIK(kin_solver->LEFT, lleg_cur_jnt_pos, lleg_traj.back(), des_jnt_pos);
      lleg_ang.push_back(des_jnt_pos);
      lleg_cur_jnt_pos = des_jnt_pos;

      rleg_ang.push_back(rleg_cur_jnt_pos);
    }
  }

  //Rewrite foot pose
  if(leg == RIGHT)
    rfoot_pose = goal_pose;
  if(leg == LEFT)
    lfoot_pose = goal_pose;

  kin_solver->deleteSolver();
  delete kin_solver;
  delete frames;

}

void op3_direct_walking::getTrajectory(leg_type leg, std::vector <Eigen::Matrix4d> &traj_vec)
{
  if(leg == RIGHT)
    traj_vec.assign(rleg_traj.begin(), rleg_traj.end());
  if(leg == LEFT)
    traj_vec.assign(lleg_traj.begin(), lleg_traj.end());

}

void op3_direct_walking::getAngles(leg_type leg, std::vector <Eigen::VectorXd> &ang_vec)
{
  if(leg == RIGHT)
    ang_vec.assign(rleg_angles.begin(), rleg_angles.end());
  if(leg == LEFT)
    ang_vec.assign(lleg_angles.begin(), lleg_angles.end());

}



}
