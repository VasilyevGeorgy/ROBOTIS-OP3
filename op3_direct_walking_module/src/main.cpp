#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <sstream>

#include <boost/thread.hpp>

#include "op3_zmp_walk.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "op3_direct_walk_node");
  ros::NodeHandle n;

  robotis_op::op3_zmp_walk *walk = new robotis_op::op3_zmp_walk();

  walk->simpleQuasistatic(walk->RIGHT, 0.03, 0.04, 10.0, 3);

  double rate = walk->getRate();

  std::vector <Eigen::VectorXd> rl_ang;
  std::vector <Eigen::VectorXd> ll_ang;

  walk->getAnglesVectors(rl_ang, ll_ang);

  std::cout<<rl_ang.size()<<std::endl;
  std::cout<<ll_ang.size()<<std::endl;

  Eigen::VectorXd r_jnt_pos(6);
  Eigen::VectorXd l_jnt_pos(6);

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

  ros::Rate loop_rate(rate);
  int count = 0;

  std::vector <std_msgs::Float64> msgs_vector;
  msgs_vector.resize(6);

  std::vector <std_msgs::Float64>::iterator it;

  while (ros::ok())
  {
    r_jnt_pos = rl_ang.at(count);
    for(it=msgs_vector.begin(); it<msgs_vector.end(); it++)
    {
      it->data = r_jnt_pos(it - msgs_vector.begin());
    }

    //ROS_INFO("%s", des_jnt_pos);
    //std::cout<<r_jnt_pos(0)<<" "<<r_jnt_pos(1)<<" "<<r_jnt_pos(2)<<" "
    //         <<r_jnt_pos(3)<<" "<<r_jnt_pos(4)<<" "<<r_jnt_pos(5)<<std::endl<<std::endl;

    r_hip_yaw_pub.publish  (msgs_vector[0]);
    r_hip_roll_pub.publish (msgs_vector[1]);
    r_hip_pitch_pub.publish(msgs_vector[2]);
    r_knee_pub.publish     (msgs_vector[3]);
    r_ank_pitch_pub.publish(msgs_vector[4]);
    r_ank_roll_pub.publish (msgs_vector[5]);

    l_jnt_pos = ll_ang.at(count);
    for(it=msgs_vector.begin(); it<msgs_vector.end(); it++)
    {
      it->data = l_jnt_pos(it - msgs_vector.begin());
    }

    //std::cout<<l_jnt_pos(0)<<" "<<l_jnt_pos(1)<<" "<<l_jnt_pos(2)<<" "
    //         <<l_jnt_pos(3)<<" "<<l_jnt_pos(4)<<" "<<l_jnt_pos(5)<<std::endl<<std::endl;

    l_hip_yaw_pub.publish  (msgs_vector[0]);
    l_hip_roll_pub.publish (msgs_vector[1]);
    l_hip_pitch_pub.publish(msgs_vector[2]);
    l_knee_pub.publish     (msgs_vector[3]);
    l_ank_pitch_pub.publish(msgs_vector[4]);
    l_ank_roll_pub.publish (msgs_vector[5]);


    ros::spinOnce();
    loop_rate.sleep();

    count++;

    if(count == rl_ang.size())
      break;
  }


  return 0;
}

