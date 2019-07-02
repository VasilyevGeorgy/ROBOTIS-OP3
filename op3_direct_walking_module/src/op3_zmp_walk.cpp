#include "op3_zmp_walk.h"

namespace robotis_op {

op3_zmp_walk::op3_zmp_walk()
{
  rate = 125.0;

  rleg_cur_jnt_pos.resize(JOINT_NUM);
  lleg_cur_jnt_pos.resize(JOINT_NUM);

  rleg_cur_jnt_pos << 0.0, 0.0, 0.01, -0.02, 0.0, 0.01;
  lleg_cur_jnt_pos << 0.0, 0.01, -0.01, 0.02, 0.01, 0.01;

}

op3_zmp_walk::~op3_zmp_walk()
{

}

void op3_zmp_walk::simpleQuasistatic(leg_type init_leg, double step_lenth, double step_height,
                                     double step_duration, unsigned int num_step)
{
  num_steps = num_step;
  if(num_steps == 0)
    return;

  //Initialization
  Eigen::Matrix4d pelvis_pose;
  pelvis_pose << 1.0, 0.0, 0.0, 0.0,
                 0.0, 1.0, 0.0, 0.0,
                 0.0, 0.0, 1.0, 0.3697,
                 0.0, 0.0, 0.0, 1.0;

  Eigen::Matrix4d rf_pose;
  rf_pose <<  1.0, 0.0, 0.0, 0.0,
                 0.0, 1.0, 0.0, -YOFFSET,
                 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 1.0;

  Eigen::Matrix4d lf_pose;
  lf_pose <<  1.0, 0.0, 0.0, 0.0,
                 0.0, 1.0, 0.0, YOFFSET,
                 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 1.0;

  this->initialization(pelvis_pose, rf_pose, lf_pose);

  //Go to initial pose
  Eigen::Matrix4d goal_pose;
  goal_pose << 1.0,  0.0,  0.0, 0.0,
               0.0,  1.0,  0.0, 0.0,
               0.0,  0.0,  1.0, 0.3,
               0.0,  0.0,  0.0, 1.0;

  //Initial CoM translation
  if(init_leg == RIGHT)
    goal_pose(1,3) = YOFFSET;
  if(init_leg == LEFT)
    goal_pose(1,3) =-YOFFSET;

  this->movePelvis(rate, 3.0, goal_pose, rl_init_step_vec, ll_init_step_vec);

  //this->getDelay(rate, 1.0);

  //Initial step
  goal_pose << 1.0,  0.0,  0.0, step_lenth * 0.25,
               0.0,  1.0,  0.0, 0.0,
               0.0,  0.0,  1.0, step_height,
               0.0,  0.0,  0.0, 1.0;

  int cur_supp_leg = 2; // Neither right, neither left

  if(init_leg == RIGHT)
  {

    goal_pose(1,3) = -YOFFSET;
    this->moveFoot(this->RIGHT, rate, step_duration * 0.125, goal_pose, rl_init_step_vec, ll_init_step_vec);
    goal_pose(0,3) = step_lenth * 0.5;
    goal_pose(2,3) = 0.0;
    this->moveFoot(this->RIGHT, rate, step_duration * 0.125, goal_pose, rl_init_step_vec, ll_init_step_vec);
    //Move CoM
    goal_pose(2,3) = 0.3;
    this->movePelvis(rate, step_duration * 0.25, goal_pose, rl_init_step_vec, ll_init_step_vec);
    cur_supp_leg = RIGHT;

  }
  if(init_leg == LEFT)
  {
    goal_pose(1,3) = YOFFSET;
    this->moveFoot(this->LEFT, rate, step_duration * 0.125, goal_pose, rl_init_step_vec, ll_init_step_vec);
    goal_pose(0,3) = step_lenth * 0.5;
    goal_pose(2,3) = 0.0;
    this->moveFoot(this->LEFT, rate, step_duration * 0.125, goal_pose, rl_init_step_vec, ll_init_step_vec);
    //Move CoM
    goal_pose(2,3) = 0.3;
    this->movePelvis(rate, step_duration * 0.25, goal_pose, rl_init_step_vec, ll_init_step_vec);
    cur_supp_leg = LEFT;

  }

  if(num_steps > 1)
  {
    //Walking loop
    if(init_leg == RIGHT) // even num_steps
    {

      goal_pose(0,3) = 0.0;
      goal_pose(1,3) = YOFFSET;
      goal_pose(2,3) = step_height;
      this->moveFoot(this->LEFT, rate, step_duration * 0.25, goal_pose, rl_walk_vec0, ll_walk_vec0);

      goal_pose(0,3) = step_lenth * 0.5;
      goal_pose(1,3) = YOFFSET;
      goal_pose(2,3) = 0.0;
      this->moveFoot(this->LEFT, rate, step_duration * 0.25, goal_pose, rl_walk_vec0, ll_walk_vec0);

      goal_pose(2,3) = 0.3;
      this->movePelvis(rate, step_duration * 0.5, goal_pose, rl_walk_vec0, ll_walk_vec0);
      cur_supp_leg = LEFT;

      if(num_steps > 2) // odd num_steps
      {
        goal_pose(0,3) = 0.0;
        goal_pose(1,3) = -YOFFSET;
        goal_pose(2,3) = step_height;
        this->moveFoot(this->RIGHT, rate, step_duration * 0.25, goal_pose, rl_walk_vec1, ll_walk_vec1);

        goal_pose(0,3) = step_lenth * 0.5;
        goal_pose(1,3) = -YOFFSET;
        goal_pose(2,3) = 0.0;
        this->moveFoot(this->RIGHT, rate, step_duration * 0.25, goal_pose, rl_walk_vec1, ll_walk_vec1);

        goal_pose(2,3) = 0.3;
        this->movePelvis(rate, step_duration * 0.5, goal_pose, rl_walk_vec1, ll_walk_vec1);
        cur_supp_leg = RIGHT;

      }
    }

    if(init_leg == LEFT) // even num_steps
    {
      goal_pose(0,3) = 0.0;
      goal_pose(1,3) = -YOFFSET;
      goal_pose(2,3) = step_height;
      this->moveFoot(this->RIGHT, rate, step_duration * 0.25, goal_pose, rl_walk_vec0, ll_walk_vec0);

      goal_pose(0,3) = step_lenth * 0.5;
      goal_pose(1,3) = -YOFFSET;
      goal_pose(2,3) = 0.0;
      this->moveFoot(this->RIGHT, rate, step_duration * 0.25, goal_pose, rl_walk_vec0, ll_walk_vec0);

      goal_pose(2,3) = 0.3;
      this->movePelvis(rate, step_duration * 0.5, goal_pose, rl_walk_vec0, ll_walk_vec0);
      cur_supp_leg = RIGHT;

      if(num_steps > 2) // odd num_steps
      {
        goal_pose(0,3) = 0.0;
        goal_pose(1,3) = YOFFSET;
        goal_pose(2,3) = step_height;
        this->moveFoot(this->LEFT, rate, step_duration * 0.25, goal_pose, rl_walk_vec1, ll_walk_vec1);

        goal_pose(0,3) = step_lenth * 0.5;
        goal_pose(1,3) = YOFFSET;
        goal_pose(2,3) = 0.0;
        this->moveFoot(this->LEFT, rate, step_duration * 0.25, goal_pose, rl_walk_vec1, ll_walk_vec1);

        goal_pose(2,3) = 0.3;
        this->movePelvis(rate, step_duration * 0.5, goal_pose, rl_walk_vec1, ll_walk_vec1);
        cur_supp_leg = LEFT;

      }
    }
  }

  ////Last step
  //if(cur_supp_leg == RIGHT)
  //{
  //  goal_pose(0,3) = -0.25 * step_lenth;
  //  goal_pose(1,3) = YOFFSET;
  //  goal_pose(2,3) = step_height;
  //  this->moveFoot(this->LEFT, rate, step_duration * 0.125, goal_pose, rl_fin_step_vec, ll_fin_step_vec);
  //
  //  goal_pose(0,3) = 0.0;
  //  goal_pose(2,3) = 0.0;
  //  this->moveFoot(this->LEFT, rate, step_duration * 0.125, goal_pose, rl_fin_step_vec, ll_fin_step_vec);
  //
  //  goal_pose(1,3) = YOFFSET;
  //  goal_pose(2,3) = 0.3;
  //  this->movePelvis(rate, step_duration * 0.25, goal_pose, rl_fin_step_vec, ll_fin_step_vec);
  //
  //}
  //if(cur_supp_leg == LEFT)
  //{
  //  goal_pose(0,3) = -0.25 * step_lenth;
  //  goal_pose(1,3) = -YOFFSET;
  //  goal_pose(2,3) = step_height;
  //  this->moveFoot(this->RIGHT, rate, step_duration * 0.125, goal_pose, rl_fin_step_vec, ll_fin_step_vec);
  //
  //  goal_pose(0,3) = 0.0;
  //  goal_pose(2,3) = 0.0;
  //  this->moveFoot(this->RIGHT, rate, step_duration * 0.125, goal_pose, rl_fin_step_vec, ll_fin_step_vec);
  //
  //  goal_pose(1,3) = -YOFFSET;
  //  goal_pose(2,3) = 0.3;
  //  this->movePelvis(rate, step_duration * 0.25, goal_pose, rl_fin_step_vec, ll_fin_step_vec);
  //
  //}


}

void op3_zmp_walk::getAnglesVectors(std::vector<Eigen::VectorXd> &rl_vec,
                                    std::vector<Eigen::VectorXd> &ll_vec)
{
  if(num_steps == 0)
    return;

  //std::vector<Eigen::VectorXd> rl_vec_;
  //std::vector<Eigen::VectorXd> ll_vec_;

  //First step
  rl_vec.assign(rl_init_step_vec.begin(), rl_init_step_vec.end());
  ll_vec.assign(ll_init_step_vec.begin(), ll_init_step_vec.end());

  //rl_vec_.insert(rl_vec.end(), rl_init_step_vec.begin(), rl_init_step_vec.end());
  //ll_vec_.insert(ll_vec.end(), ll_init_step_vec.begin(), ll_init_step_vec.end());

  if(num_steps > 1)
  {
    for(unsigned int i=2; i<=num_steps; i++)
    {
      if(i%2 == 0)
      {
        rl_vec.insert(rl_vec.end(), rl_walk_vec0.begin(), rl_walk_vec0.end());
        ll_vec.insert(ll_vec.end(), ll_walk_vec0.begin(), ll_walk_vec0.end());
      }
      else{
        rl_vec.insert(rl_vec.end(), rl_walk_vec1.begin(), rl_walk_vec1.end());
        ll_vec.insert(ll_vec.end(), ll_walk_vec1.begin(), ll_walk_vec1.end());
      }
    }

  }

  //Last step
  //rl_vec.insert(rl_vec.end(), rl_fin_step_vec.begin(), rl_fin_step_vec.end());
  //ll_vec.insert(ll_vec.end(), ll_fin_step_vec.begin(), ll_fin_step_vec.end());

  //rl_vec = rl_vec_;
  //ll_vec = ll_vec_;
  //std::cout<<rl_vec_.size()<<std::endl;
  //std::cout<<ll_vec_.size()<<std::endl;


}


double op3_zmp_walk::getRate()
{
  return rate;
}

}
