#ifndef OP3_ZMP_WALK_H
#define OP3_ZMP_WALK_H

#include "../include/op3_direct_walking_module/op3_direct_walking.h"

namespace robotis_op {

class op3_zmp_walk : public robotis_op::op3_direct_walking
{
public:
  op3_zmp_walk();
  virtual ~op3_zmp_walk();

  void simpleQuasistatic(leg_type init_leg, double step_lenth, double step_height,
                         double step_duration, unsigned int num_step);

  void getAnglesVectors(std::vector <Eigen::VectorXd> &rl_vec,
                        std::vector <Eigen::VectorXd> &ll_vec);

  double getRate();

private:
  double rate; // control rate
  unsigned int num_steps;

  Eigen::Matrix4d pelvis_pose;
  Eigen::Matrix4d rfoot_pose;
  Eigen::Matrix4d lfoot_pose;

  std::vector <Eigen::Matrix4d> rleg_traj;
  std::vector <Eigen::Matrix4d> lleg_traj;

  Eigen::VectorXd rleg_cur_jnt_pos;
  Eigen::VectorXd lleg_cur_jnt_pos;

  std::vector <Eigen::VectorXd> rl_init_step_vec;
  std::vector <Eigen::VectorXd> ll_init_step_vec;

  std::vector <Eigen::VectorXd> rl_walk_vec0;
  std::vector <Eigen::VectorXd> ll_walk_vec0;
  std::vector <Eigen::VectorXd> rl_walk_vec1;
  std::vector <Eigen::VectorXd> ll_walk_vec1;

  std::vector <Eigen::VectorXd> rl_fin_step_vec;
  std::vector <Eigen::VectorXd> ll_fin_step_vec;



};

}

#endif // OP3_ZMP_WALK_H
