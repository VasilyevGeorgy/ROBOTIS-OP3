#ifndef KINSOLVER_H
#define KINSOLVER_H

#include <eigen3/Eigen/Eigen>

#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#include <iostream>


#define JOINT_NUM 6
#define D2R M_PI/180.0
#define R2D 180.0/M_PI

namespace  robotis_op{

class KinSolver
{

public:
  KinSolver();
  virtual ~KinSolver();

  enum leg_type{
    RIGHT,
    LEFT
  };

  bool initSolver(Eigen::Matrix4d base_pose);
  void deleteSolver();
  bool solveFK(leg_type leg, Eigen::VectorXd cur_jnt_pos, Eigen::Matrix4d &des_pose);
  bool solveIK(leg_type leg, Eigen::VectorXd cur_jnt_pos, Eigen::Matrix4d des_pose, Eigen::VectorXd &des_jnt_pos);

private:
  void initChains(KDL::Frame pelvis_frame);
  void deleteChains();

  //Joint limits
  KDL::JntArray rleg_min_pos_limit;
  KDL::JntArray rleg_max_pos_limit;
  KDL::JntArray lleg_min_pos_limit;
  KDL::JntArray lleg_max_pos_limit;

  //Kinematic chains and solvers
  KDL::Chain *rleg_chain;
  KDL::Chain *lleg_chain;
  //Right leg
  KDL::ChainFkSolverPos_recursive *rleg_fk_solver;
  KDL::ChainIkSolverVel_pinv      *rleg_ik_vel_solver;
  KDL::ChainIkSolverPos_NR_JL     *rleg_ik_pos_solver;
  //Left leg
  KDL::ChainFkSolverPos_recursive *lleg_fk_solver;
  KDL::ChainIkSolverVel_pinv      *lleg_ik_vel_solver;
  KDL::ChainIkSolverPos_NR_JL     *lleg_ik_pos_solver;


};

}

#endif // KINSOLVER_H
