#ifndef OP3_DIRECT_WALKING_H
#define OP3_DIRECT_WALKING_H

#include "kinsolver.h"
#include "frames.h"

namespace  robotis_op{

class op3_direct_walking
{
public:
  op3_direct_walking();
  virtual ~op3_direct_walking();

  enum leg_type{
    RIGHT,
    LEFT
  };

  void initialization(Eigen::Matrix4d pel_pos, Eigen::Matrix4d rl_pos, Eigen::Matrix4d ll_pos);

  void getDelay(double operating_rate, double duration,
                std::vector <Eigen::VectorXd> &rleg_ang, std::vector <Eigen::VectorXd> &lleg_ang);
  void movePelvis(double operating_rate, double duration, Eigen::Matrix4d goal_pose,
                  std::vector <Eigen::VectorXd> &rleg_ang, std::vector <Eigen::VectorXd> &lleg_ang);
  void moveFoot(leg_type leg, double operating_rate, double duration, Eigen::Matrix4d goal_pose,
                std::vector<Eigen::VectorXd> &rleg_ang, std::vector<Eigen::VectorXd> &lleg_ang);

  void getTrajectory(leg_type leg, std::vector <Eigen::Matrix4d> &traj_vec);
  void getAngles(leg_type leg, std::vector <Eigen::VectorXd> &ang_vec);

private:

  Eigen::Matrix4d pelvis_pose;
  Eigen::Matrix4d rfoot_pose;
  Eigen::Matrix4d lfoot_pose;

  std::vector <Eigen::Matrix4d> rleg_traj;
  std::vector <Eigen::Matrix4d> lleg_traj;

  Eigen::VectorXd rleg_cur_jnt_pos;
  Eigen::VectorXd lleg_cur_jnt_pos;

  std::vector <Eigen::VectorXd> rleg_angles;
  std::vector <Eigen::VectorXd> lleg_angles;

};

}

#endif // OP3_DIRECT_WALKING_H
