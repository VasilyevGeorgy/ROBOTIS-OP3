#ifndef OP3_DIRECT_WALKING_H
#define OP3_DIRECT_WALKING_H

#include "../include/op3_direct_walking_module/kinsolver.h"
#include "trajectorygenerator.h"

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
  void pevlisTranslation(double operating_rate, double duration, Eigen::Matrix4d goal_pose);

private:

  Eigen::Matrix4d pelvis_pose;
  Eigen::Matrix4d rfoot_pose;
  Eigen::Matrix4d lfoot_pose;

};

}

#endif // OP3_DIRECT_WALKING_H
