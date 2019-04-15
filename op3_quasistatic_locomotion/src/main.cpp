#include "op3_quasistatic_locomotion.h"

int main (int argc, char **argv){

  ros::init(argc,argv,"set_initial_pose");

  op3_quasistatic_locomotion move;

  move.launchManager();

  KDL::Frame goalPose = KDL::Frame(KDL::Rotation::RPY(0.0,0.0,0.0),
                                   KDL::Vector(0.0, 0.0, 0.3));

  op3_quasistatic_locomotion::stepParam sp;
  sp.freq           = 500.0; // HZ
  sp.num_of_steps   = 5.0;
  sp.init_leg       = "RighT";
  sp.step_length    = 30.0;   //mm
  sp.step_duration  = 8.0;  //sec
  sp.step_clearance = 35.0; //mm

  //move.quasiStaticPlaner(goalPose, sp);
  //move.locomotion(sp);

  Eigen::VectorXd rleg_joint_pos_;
  Eigen::VectorXd lleg_joint_pos_;
  rleg_joint_pos_.resize(JOINT_NUM);
  lleg_joint_pos_.resize(JOINT_NUM);

  //move.goToInitialPose(goalPose, sp);
  move.quasiStaticPlaner(goalPose, sp);
  move.locomotion(sp);

  return 0;
}
