#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <sstream>

#include <boost/thread.hpp>

#include "op3_direct_walking.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "op3_direct_walk_node");
  ros::NodeHandle n;

/*
  robotis_op::KinSolver *kin_solver = new robotis_op::KinSolver();

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

  kin_solver->initSolver(pelvis_pose);

  robotis_op::TrajectoryGenerator *tr_gen = new robotis_op::TrajectoryGenerator();
  tr_gen->initialize(pelvis_pose, rf_pose, lf_pose);

  Eigen::Matrix4d goal_pose;
  goal_pose << 0.9848077,  0.0,  0.1736483, 0.0,
               0.0000000,  1.0,  0.0000000, 0.0,
              -0.1736483,  0.0,  0.9848077, 0.215,
               0.0,        0.0,  0.0,       1.0;

  //goal_pose << 1.0,  0.0,  0.0, 0.0,
  //             0.0,  1.0,  0.0, 0.0,
  //             0.0,  0.0,  1.0, 0.215,
  //             0.0,  0.0,  0.0, 1.0;

  double rate = 1.0/0.008; // ms

  tr_gen->pevlisTranslation(rate, 2.0, goal_pose);

  std::vector <Eigen::Matrix4d> r_traj;
  std::vector <Eigen::Matrix4d> l_traj;

  tr_gen->getTrajectory(tr_gen->RIGHT, r_traj);
  tr_gen->getTrajectory(tr_gen->LEFT, l_traj);

  //for (unsigned i=0; i< r_traj.size(); i++)
  //  std::cout<<r_traj.at(i)<<std::endl;

  Eigen::VectorXd r_cur_jnt_pos(6);
  r_cur_jnt_pos << 0.0, 0.0, 0.1, -0.02, 0.0, 0.01;
  Eigen::VectorXd l_cur_jnt_pos(6);
  l_cur_jnt_pos << 0.0, 0.01, -0.01, 0.01, 0.01, 0.01;

  std::vector <Eigen::VectorXd> r_des_jnt_pos;
  std::vector <Eigen::VectorXd> l_des_jnt_pos;

  //Vectors for desired pose
  Eigen::VectorXd r_jnt_pos(JOINT_NUM);
  Eigen::VectorXd l_jnt_pos(JOINT_NUM);

  for(unsigned int i=0; i < r_traj.size(); i++)
  {
    kin_solver->solveIK(kin_solver->RIGHT, r_cur_jnt_pos, r_traj.at(i), r_jnt_pos);
    r_des_jnt_pos.push_back(r_jnt_pos);
    r_cur_jnt_pos = r_jnt_pos;

    kin_solver->solveIK(kin_solver->LEFT, l_cur_jnt_pos, l_traj.at(i), l_jnt_pos);
    l_des_jnt_pos.push_back(l_jnt_pos);
    l_cur_jnt_pos = l_jnt_pos;
  }

  kin_solver->deleteSolver();
  delete tr_gen;

  robotis_op::TrajectoryGenerator *tr_gen1 = new robotis_op::TrajectoryGenerator();
  tr_gen1->initialize(goal_pose, rf_pose, lf_pose);
  kin_solver->initSolver(goal_pose);

  goal_pose << 1.0,  0.0,  0.0, 0.0,
               0.0,  1.0,  0.0, 0.0,
               0.0,  0.0,  1.0, 0.3,
               0.0,  0.0,  0.0, 1.0;

  tr_gen1->pevlisTranslation(rate, 2.0, goal_pose);

  std::vector <Eigen::Matrix4d> r_traj1;
  std::vector <Eigen::Matrix4d> l_traj1;

  tr_gen1->getTrajectory(tr_gen1->RIGHT, r_traj1);
  tr_gen1->getTrajectory(tr_gen1->LEFT, l_traj1);

  for(unsigned int i=0; i < r_traj1.size(); i++)
  {
    kin_solver->solveIK(kin_solver->RIGHT, r_cur_jnt_pos, r_traj1.at(i), r_jnt_pos);
    r_des_jnt_pos.push_back(r_jnt_pos);
    r_cur_jnt_pos = r_jnt_pos;

    kin_solver->solveIK(kin_solver->LEFT, l_cur_jnt_pos, l_traj1.at(i), l_jnt_pos);
    l_des_jnt_pos.push_back(l_jnt_pos);
    l_cur_jnt_pos = l_jnt_pos;
  }

  /*

  std::vector <Eigen::Matrix4d> r_traj;
  std::vector <Eigen::Matrix4d> l_traj;

  tr_gen->getTrajectory(tr_gen->RIGHT, r_traj);
  tr_gen->getTrajectory(tr_gen->LEFT, l_traj);

  //for (unsigned i=0; i< r_traj.size(); i++)
  //  std::cout<<r_traj.at(i)<<std::endl;

  Eigen::VectorXd r_cur_jnt_pos(6);
  r_cur_jnt_pos << 0.0, 0.0, 0.1, -0.02, 0.0, 0.01;
  Eigen::VectorXd l_cur_jnt_pos(6);
  l_cur_jnt_pos << 0.0, 0.01, -0.01, 0.01, 0.01, 0.01;

  std::vector <Eigen::VectorXd> r_des_jnt_pos;
  std::vector <Eigen::VectorXd> l_des_jnt_pos;

  //Vectors for desired pose
  Eigen::VectorXd r_jnt_pos(JOINT_NUM);
  Eigen::VectorXd l_jnt_pos(JOINT_NUM);

  for(unsigned int i=0; i < r_traj.size(); i++)
  {
    kin_solver->solveIK(kin_solver->RIGHT, r_cur_jnt_pos, r_traj.at(i), r_jnt_pos);
    r_des_jnt_pos.push_back(r_jnt_pos);
    r_cur_jnt_pos = r_jnt_pos;

    kin_solver->solveIK(kin_solver->LEFT, l_cur_jnt_pos, l_traj.at(i), l_jnt_pos);
    l_des_jnt_pos.push_back(l_jnt_pos);
    l_cur_jnt_pos = l_jnt_pos;
  }


  kin_solver->deleteSolver();
  delete kin_solver;
  delete tr_gen1;

  // Initialize ROS publishers
  ros::Publisher r_hip_yaw_pub   = n.advertise<std_msgs::Float64>("/robotis_op3/r_hip_yaw_position/command", 1);
  ros::Publisher l_hip_yaw_pub   = n.advertise<std_msgs::Float64>("/robotis_op3/l_hip_yaw_position/command", 1);

  ros::Publisher r_hip_roll_pub  = n.advertise<std_msgs::Float64>("/robotis_op3/r_hip_roll_position/command", 1);
  ros::Publisher l_hip_roll_pub  = n.advertise<std_msgs::Float64>("/robotis_op3/l_hip_roll_position/command", 1);

  ros::Publisher r_hip_pitch_pub = n.advertise<std_msgs::Float64>("/robotis_op3/r_hip_pitch_position/command", 1);
  ros::Publisher l_hip_pitch_pub = n.advertise<std_msgs::Float64>("/robotis_op3/l_hip_pitch_position/command", 1);

  ros::Publisher r_knee_pub      = n.advertise<std_msgs::Float64>("/robotis_op3/r_knee_position/command", 1);
  ros::Publisher l_knee_pub      = n.advertise<std_msgs::Float64>("/robotis_op3/l_knee_position/command", 1);

  ros::Publisher r_ank_pitch_pub = n.advertise<std_msgs::Float64>("/robotis_op3/r_ank_pitch_position/command", 1);
  ros::Publisher l_ank_pitch_pub = n.advertise<std_msgs::Float64>("/robotis_op3/l_ank_pitch_position/command", 1);

  ros::Publisher r_ank_roll_pub  = n.advertise<std_msgs::Float64>("/robotis_op3/r_ank_roll_position/command", 1);
  ros::Publisher l_ank_roll_pub  = n.advertise<std_msgs::Float64>("/robotis_op3/l_ank_roll_position/command", 1);

  ros::Rate loop_rate(rate/5.0);

  int count = 0;

  while (ros::ok())
  {

    std_msgs::Float64 angle0_msg;
    std_msgs::Float64 angle1_msg;
    std_msgs::Float64 angle2_msg;
    std_msgs::Float64 angle3_msg;
    std_msgs::Float64 angle4_msg;
    std_msgs::Float64 angle5_msg;

    r_jnt_pos = r_des_jnt_pos.at(count);

    angle0_msg.data = r_jnt_pos(0);
    angle1_msg.data = r_jnt_pos(1);
    angle2_msg.data = r_jnt_pos(2);
    angle3_msg.data = r_jnt_pos(3);
    angle4_msg.data = r_jnt_pos(4);
    angle5_msg.data = r_jnt_pos(5);

    //ROS_INFO("%s", des_jnt_pos);
    //std::cout<<r_jnt_pos(0)<<" "<<r_jnt_pos(1)<<" "<<r_jnt_pos(2)<<" "
    //         <<r_jnt_pos(3)<<" "<<r_jnt_pos(4)<<" "<<r_jnt_pos(5)<<std::endl<<std::endl;

    r_hip_yaw_pub.publish(angle0_msg);
    r_hip_roll_pub.publish(angle1_msg);
    r_hip_pitch_pub.publish(angle2_msg);
    r_knee_pub.publish(angle3_msg);
    r_ank_pitch_pub.publish(angle4_msg);
    r_ank_roll_pub.publish(angle5_msg);

    l_jnt_pos = l_des_jnt_pos.at(count);

    angle0_msg.data = l_jnt_pos(0);
    angle1_msg.data = l_jnt_pos(1);
    angle2_msg.data = l_jnt_pos(2);
    angle3_msg.data = l_jnt_pos(3);
    angle4_msg.data = l_jnt_pos(4);
    angle5_msg.data = l_jnt_pos(5);

    //std::cout<<l_jnt_pos(0)<<" "<<l_jnt_pos(1)<<" "<<l_jnt_pos(2)<<" "
    //         <<l_jnt_pos(3)<<" "<<l_jnt_pos(4)<<" "<<l_jnt_pos(5)<<std::endl<<std::endl;

    l_hip_yaw_pub.publish(angle0_msg);
    l_hip_roll_pub.publish(angle1_msg);
    l_hip_pitch_pub.publish(angle2_msg);
    l_knee_pub.publish(angle3_msg);
    l_ank_pitch_pub.publish(angle4_msg);
    l_ank_roll_pub.publish(angle5_msg);


    ros::spinOnce();
    loop_rate.sleep();

    count++;

    if(count == r_des_jnt_pos.size())
      break;
  }

*/

  return 0;
}

