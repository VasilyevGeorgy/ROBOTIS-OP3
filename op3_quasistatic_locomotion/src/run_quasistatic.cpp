#include <ros/ros.h>
#include <std_msgs/String.h>
#include "op3_online_walking_module_msgs/FootStepCommand.h"

int main(int argc, char **argv){

  ros::init(argc,argv,"run_quasistatic");

  ros::NodeHandle nh;
  // ROS Publishers
  ros::Publisher ctrl_module_pub = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);
  ros::Publisher param_pub = nh.advertise<op3_online_walking_module_msgs::FootStepCommand>("/robotis/quasistatic/step_params", 1);

  std_msgs::String ctrl_module_msg;
  ctrl_module_msg.data = "quasistatic_module";

  op3_online_walking_module_msgs::FootStepCommand param_msg;
  param_msg.step_num    = 10;
  param_msg.start_leg   = "right";
  param_msg.step_length = 30.0;
  param_msg.step_time   = 2.0;
  param_msg.side_length = 20.0;

  ros::Rate freq(1000);

  while(ros::ok()){

    ctrl_module_pub.publish(ctrl_module_msg);
    param_pub.publish(param_msg);

    ros::spinOnce();
    freq.sleep();

  }

  return 0;
}
