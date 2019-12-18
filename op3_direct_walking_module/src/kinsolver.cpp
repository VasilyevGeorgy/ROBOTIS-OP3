#include "../include/op3_direct_walking_module/kinsolver.h"

namespace robotis_op {

KinSolver::KinSolver()
{
  rleg_min_pos_limit.resize(JOINT_NUM);
  rleg_max_pos_limit.resize(JOINT_NUM);
  lleg_min_pos_limit.resize(JOINT_NUM);
  lleg_max_pos_limit.resize(JOINT_NUM);

  for (int i=0; i < JOINT_NUM; i++)
  {
    rleg_min_pos_limit(i) = 0.0;
    rleg_max_pos_limit(i) = 0.0;

    lleg_min_pos_limit(i) = 0.0;
    lleg_max_pos_limit(i) = 0.0;
  }

}

KinSolver::~KinSolver(){

}

bool KinSolver::initSolver(Eigen::Matrix4d base_pose)
{
  //Convert Eigen frame to KDL
  Eigen::MatrixXd r(3,3);
  r << base_pose.block(0,0,3,3); // get rotation matrix
  double x = base_pose(0,3);     // get position vectors
  double y = base_pose(1,3);
  double z = base_pose(2,3);

  KDL::Frame base_frame_ = KDL::Frame(KDL::Rotation(r(0,0), r(0,1), r(0,2),
                                                    r(1,0), r(1,1), r(1,2),
                                                    r(2,0), r(2,1), r(2,2)),
                                     KDL::Vector(x,y,z));

  this->initChains(base_frame_);

  //Set joint limits
  std::vector<double> rleg_min_pos_limit_, rleg_max_pos_limit_;
  rleg_min_pos_limit_.push_back(-90.0);  rleg_max_pos_limit_.push_back(90.0); // hip_y
  rleg_min_pos_limit_.push_back(-90.0);	 rleg_max_pos_limit_.push_back(90.0); // hip_r
  rleg_min_pos_limit_.push_back(-90.0);  rleg_max_pos_limit_.push_back(90.0); // hip_p
  rleg_min_pos_limit_.push_back(-180.0); rleg_max_pos_limit_.push_back(0.01); // kn_p
  rleg_min_pos_limit_.push_back(-90.0);	 rleg_max_pos_limit_.push_back(90.0); // an_p
  rleg_min_pos_limit_.push_back(-90.0);  rleg_max_pos_limit_.push_back(90.0); // an_r

  std::vector<double> lleg_min_pos_limit_, lleg_max_pos_limit_;
  lleg_min_pos_limit_.push_back(-90.0); lleg_max_pos_limit_.push_back(90.0);  // hip_y
  lleg_min_pos_limit_.push_back(-90.0);	lleg_max_pos_limit_.push_back(90.0);  // hip_r
  lleg_min_pos_limit_.push_back(-90.0); lleg_max_pos_limit_.push_back(90.0);  // hip_p
  lleg_min_pos_limit_.push_back(-0.01);	lleg_max_pos_limit_.push_back(180.0); // kn_p
  lleg_min_pos_limit_.push_back(-90.0);	lleg_max_pos_limit_.push_back(90.0);  // an_p
  lleg_min_pos_limit_.push_back(-90.0); lleg_max_pos_limit_.push_back(90.0);  // an_r

  for (int i=0; i<JOINT_NUM; i++)
  {
    rleg_min_pos_limit(i) = rleg_min_pos_limit_[i]*D2R; //D2R - degrees to radians
    rleg_max_pos_limit(i) = rleg_max_pos_limit_[i]*D2R;

    lleg_min_pos_limit(i) = lleg_min_pos_limit_[i]*D2R; //D2R - degrees to radians
    lleg_max_pos_limit(i) = lleg_max_pos_limit_[i]*D2R;

  }

  //Setup solvers
  rleg_fk_solver = new KDL::ChainFkSolverPos_recursive(*rleg_chain);
  rleg_ik_vel_solver  = new KDL::ChainIkSolverVel_pinv(*rleg_chain);
  rleg_ik_pos_solver = new KDL::ChainIkSolverPos_NR_JL(*rleg_chain,
                                                           rleg_min_pos_limit, rleg_max_pos_limit,
                                                           *rleg_fk_solver,
                                                           *rleg_ik_vel_solver
                                                           );

  lleg_fk_solver = new KDL::ChainFkSolverPos_recursive(*lleg_chain);
  lleg_ik_vel_solver  = new KDL::ChainIkSolverVel_pinv(*lleg_chain);
  lleg_ik_pos_solver = new KDL::ChainIkSolverPos_NR_JL(*lleg_chain,
                                                           lleg_min_pos_limit, lleg_max_pos_limit,
                                                           *lleg_fk_solver,
                                                           *lleg_ik_vel_solver
                                                           );


}

void KinSolver::initChains(KDL::Frame pelvis_frame)
{
  // Set Kinematics Tree
  // Right Leg Chain

  rleg_chain = new KDL::Chain;
  lleg_chain = new KDL::Chain;

  rleg_chain->addSegment(KDL::Segment("base",
                                     KDL::Joint(KDL::Joint::None),
                                     pelvis_frame,
                                     KDL::RigidBodyInertia(0.0,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  rleg_chain->addSegment(KDL::Segment("pelvis",
                                     KDL::Joint(KDL::Joint::None),
                                     KDL::Frame(KDL::Vector(0.0, -0.035, -0.0907)),
                                     KDL::RigidBodyInertia(0.72235,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  rleg_chain->addSegment(KDL::Segment("r_hip_yaw",
                                     KDL::Joint("minus_RotZ", KDL::Vector(0,0,0), KDL::Vector(0,0,-1), KDL::Joint::RotAxis),
                                     KDL::Frame(KDL::Vector(0.000, 0.000, -0.0285)),
                                     KDL::RigidBodyInertia(0.01181,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  rleg_chain->addSegment(KDL::Segment("r_leg_hip_r",
                                     KDL::Joint("minus_RotX", KDL::Vector(0,0,0), KDL::Vector(-1,0,0), KDL::Joint::RotAxis),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                     KDL::RigidBodyInertia(0.17886,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  rleg_chain->addSegment(KDL::Segment("r_leg_hip_p",
                                     KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, -0.11)),
                                     KDL::RigidBodyInertia(0.11543,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  rleg_chain->addSegment(KDL::Segment("r_leg_kn_p",
                                     KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, -0.11)),
                                     KDL::RigidBodyInertia(0.04015,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  rleg_chain->addSegment(KDL::Segment("r_leg_an_p",
                                     KDL::Joint(KDL::Joint::RotY),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                     KDL::RigidBodyInertia(0.17886,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  rleg_chain->addSegment(KDL::Segment("r_leg_an_r",
                                     KDL::Joint(KDL::Joint::RotX),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, -0.0305)),
                                     KDL::RigidBodyInertia(0.06934,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  rleg_chain->addSegment(KDL::Segment("r_leg_end",
                                     KDL::Joint(KDL::Joint::None),
                                     KDL::Frame(KDL::Vector(0.0 , 0.0 , 0.0)),
                                     KDL::RigidBodyInertia(0.0,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );

  // Left Leg Chain
  lleg_chain->addSegment(KDL::Segment("base",
                                     KDL::Joint(KDL::Joint::None),
                                     pelvis_frame,
                                     KDL::RigidBodyInertia(0.0,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  lleg_chain->addSegment(KDL::Segment("pelvis",
                                     KDL::Joint(KDL::Joint::None),
                                     KDL::Frame(KDL::Vector(0.0, 0.035, -0.0907)),
                                     KDL::RigidBodyInertia(0.72235,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  lleg_chain->addSegment(KDL::Segment("l_leg_hip_y",
                                     KDL::Joint("minus_RotZ", KDL::Vector(0,0,0), KDL::Vector(0,0,-1), KDL::Joint::RotAxis),
                                     KDL::Frame(KDL::Vector(0.000, 0.000, -0.0285)),
                                     KDL::RigidBodyInertia(0.01181,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  lleg_chain->addSegment(KDL::Segment("l_leg_hip_r",
                                     KDL::Joint("minus_RotX", KDL::Vector(0,0,0), KDL::Vector(-1,0,0), KDL::Joint::RotAxis),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                     KDL::RigidBodyInertia(0.17886,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  lleg_chain->addSegment(KDL::Segment("l_leg_hip_p",
                                     KDL::Joint(KDL::Joint::RotY),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, -0.11)),
                                     KDL::RigidBodyInertia(0.11543,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  lleg_chain->addSegment(KDL::Segment("l_leg_kn_p",
                                     KDL::Joint(KDL::Joint::RotY),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, -0.11)),
                                     KDL::RigidBodyInertia(0.04015,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  lleg_chain->addSegment(KDL::Segment("l_leg_an_p",
                                     KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                     KDL::RigidBodyInertia(0.17886,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  lleg_chain->addSegment(KDL::Segment("l_leg_an_r",
                                     KDL::Joint(KDL::Joint::RotX),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, -0.0305)),
                                     KDL::RigidBodyInertia(0.06934,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  lleg_chain->addSegment(KDL::Segment("l_leg_end",
                                     KDL::Joint(KDL::Joint::None),
                                     KDL::Frame(KDL::Vector(0.0 , 0.0 , 0.0)),
                                     KDL::RigidBodyInertia(0.0,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
}

bool KinSolver::solveIK(leg_type leg, Eigen::VectorXd cur_jnt_pos,
                        Eigen::Matrix4d des_pose, Eigen::VectorXd &des_jnt_pos)
{

  //Convert Eigen vectors to KDL
  KDL::JntArray cur_joint_pos_;
  KDL::JntArray des_joint_pos_;

  cur_joint_pos_.resize(JOINT_NUM);
  des_joint_pos_.resize(JOINT_NUM);

  for (int i=0; i < JOINT_NUM; i++){
    cur_joint_pos_(i) = cur_jnt_pos(i);
    des_joint_pos_(i) = cur_jnt_pos(i);
  }
  //Convert Eigen matrix to KDL frame
  Eigen::MatrixXd r(3,3);
  r << des_pose.block(0,0,3,3); // get rotation matrix
  double x = des_pose(0,3);     // get position vectors
  double y = des_pose(1,3);
  double z = des_pose(2,3);

  KDL::Frame des_frame_ = KDL::Frame(KDL::Rotation(r(0,0), r(0,1), r(0,2),
                                                   r(1,0), r(1,1), r(1,2),
                                                   r(2,0), r(2,1), r(2,2)),
                                     KDL::Vector(x,y,z));
  int ik_error = 0;

  if(leg == RIGHT)
  {
    ik_error = rleg_ik_pos_solver->CartToJnt(cur_joint_pos_, des_frame_, des_joint_pos_);

    if(ik_error!=0){
      //std::cout<<"RIGHT LEG IK ERROR: "<<rleg_ik_pos_solver->strError(ik_error)<<std::endl;
      return false;
    }
  }
  else{
    if(leg == LEFT)
    {
      ik_error = lleg_ik_pos_solver->CartToJnt(cur_joint_pos_, des_frame_, des_joint_pos_);

      if(ik_error!=0){
        //std::cout<<"LEFT LEG IK ERROR: "<<lleg_ik_pos_solver->strError(ik_error)<<std::endl;
        return false;
      }
    }
    else{
      //std::cout<<"IK ERROR: wrong leg variable was declared"<<std::endl;
      return false;
    }
  }

  //Convert KDL joint array to Eigen vector
  for (int i=0; i < JOINT_NUM; i++)
    des_jnt_pos(i) = des_joint_pos_(i);

  return true;

}

bool KinSolver::solveFK(leg_type leg, Eigen::VectorXd cur_jnt_pos, Eigen::Matrix4d &des_pose)
{
  //Convert Eigen vectors to KDL
  KDL::JntArray cur_joint_pos_;
  KDL::Frame des_frame_;

  cur_joint_pos_.resize(JOINT_NUM);

  for (int i=0; i < JOINT_NUM; i++){
    cur_joint_pos_(i) = cur_jnt_pos(i);
  }

  if(leg == RIGHT){
     int fk_error = rleg_fk_solver->JntToCart(cur_joint_pos_,des_frame_);

     if (fk_error < 0){
       //std::cout<<"RIGHT LEG FK ERROR"<<std::endl;
       return false;
     }
  }
  else{
    if(leg == LEFT){

      int fk_error = lleg_fk_solver->JntToCart(cur_joint_pos_,des_frame_);

      if (fk_error < 0){
        //std::cout<<"LEFT LEG FK ERROR"<<std::endl;
        return false;
      }
    }
    else{
      std::cout<<"FK ERROR: wrong leg variable was declared"<<std::endl;
      return false;
    }
  }

  des_pose << des_frame_.operator()(0,0), des_frame_.operator()(0,1), des_frame_.operator()(0,2), des_frame_.operator()(0,3),
              des_frame_.operator()(1,0), des_frame_.operator()(1,1), des_frame_.operator()(1,2), des_frame_.operator()(1,3),
              des_frame_.operator()(2,0), des_frame_.operator()(2,1), des_frame_.operator()(2,2), des_frame_.operator()(2,3),
              des_frame_.operator()(3,0), des_frame_.operator()(3,1), des_frame_.operator()(3,2), des_frame_.operator()(3,3);

  return true;

}

void KinSolver::deleteChains()
{
  delete rleg_chain;
  delete lleg_chain;
}

void KinSolver::deleteSolver(){

  delete rleg_fk_solver;
  delete rleg_ik_vel_solver;
  delete rleg_ik_pos_solver;

  delete lleg_fk_solver;
  delete lleg_ik_vel_solver;
  delete lleg_ik_pos_solver;

  this->deleteChains();

}

}

