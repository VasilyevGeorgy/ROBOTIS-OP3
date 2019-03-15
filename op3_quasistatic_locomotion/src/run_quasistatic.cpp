#include <iostream>
#include <vector>
#include <algorithm>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "op3_online_walking_module_msgs/FootStepCommand.h"

double get_value(std::string &cur_str);
void   get_str(std::string &cur_str);

int main(int argc, char **argv){

  std::vector<std::string> args;
  for (int i=0; i<argc; i++){
    std::string str = argv[i];
    std::transform(str.begin(),str.end(),str.begin(), ::tolower);
    args.push_back(str);
  }

  if(args.size()>6){
    ROS_ERROR("Too many parameters!");
    return 1;
  }

  int step_num           = 0;
  std::string init_leg   = "right";
  double step_len        = 0.0;
  double step_duration   = 0.0;
  double foot_clearance  = 0.0;

  for(std::vector<std::string>::iterator it = args.begin(); it != args.end(); it++){

    std::string cur_str = *it;

    if(cur_str.find("num") != std::string::npos){
      step_num = get_value(cur_str);
      //std::cout<<"step_num: "<<step_num<<std::endl;
      continue;
    }

    if((cur_str.find("init") != std::string::npos || cur_str.find("start") != std::string::npos)
                && (cur_str.find("leg") != std::string::npos || cur_str.find("step") != std::string::npos)){
      get_str(cur_str);
      if (!cur_str.empty())
        init_leg = cur_str;
      //std::cout<<"init_leg: "<<init_leg<<std::endl;
      continue;
    }

    if(cur_str.find("len") != std::string::npos){
      step_len = get_value(cur_str);
      //std::cout<<"step_len: "<<step_len<<std::endl;
      continue;
    }

    if((cur_str.find("time") != std::string::npos) || (cur_str.find("duration") != std::string::npos)){
      step_duration = get_value(cur_str);
      //std::cout<<"step_duration: "<<step_duration<<std::endl;
      continue;
    }

    if((cur_str.find("clearance") != std::string::npos) || (cur_str.find("hei") != std::string::npos)){
      foot_clearance = get_value(cur_str);
      //std::cout<<"foot_clearance: "<<foot_clearance<<std::endl;
      continue;
    }

    //std::cout<<*it<<std::endl;

  }

  ros::init(argc,argv,"run_quasistatic");

  ros::NodeHandle nh;
  // ROS Publishers
  ros::Publisher ctrl_module_pub = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);
  ros::Publisher param_pub = nh.advertise<op3_online_walking_module_msgs::FootStepCommand>("/robotis/quasistatic/step_params", 1);

  std_msgs::String ctrl_module_msg;
  ctrl_module_msg.data = "quasistatic_module";

  op3_online_walking_module_msgs::FootStepCommand param_msg;

  if(step_num > 0)
    param_msg.step_num    = step_num;
  else
    param_msg.step_num    = 1;

  if(init_leg == "left")
    param_msg.start_leg   = init_leg;
  else{
    if(init_leg == "right")
      param_msg.start_leg  = init_leg;
    else
      param_msg.start_leg  = "right";

  }

  if(step_len > 0)
    param_msg.step_length = step_len;
  else
    param_msg.step_length = 10; // mm

  if(step_duration > 0)
    param_msg.step_time   = step_duration;
  else
    param_msg.step_time   = 6; // sec

  if(foot_clearance > 0)
    param_msg.side_length = foot_clearance;
  else
    param_msg.side_length = 10.0; // mm

  ROS_INFO("step_num: %d", param_msg.step_num);
  ROS_INFO("init_leg: %s", (param_msg.start_leg).c_str());
  ROS_INFO("step_len: %f", param_msg.step_length);
  ROS_INFO("step_duration: %f", param_msg.step_time);
  ROS_INFO("foot_clearance: %f", param_msg.side_length);

  ros::Rate freq(1000);
  ROS_WARN("Start quasistatic locomotion!");

  while(ros::ok()){

    ctrl_module_pub.publish(ctrl_module_msg);
    param_pub.publish(param_msg);

    ros::spinOnce();
    freq.sleep();

  }

  return 0;
}

double get_value(std::string &cur_str){

      std::size_t pos = cur_str.find("=");
      std::string str = cur_str.substr(pos+1); // get value after '='

      std::istringstream instr;
      instr.str(str);
      double value = 0.0;
      instr >> value; // convert str to double

      return value;

}

void get_str(std::string &cur_str){

      std::size_t pos = cur_str.find("=");
      cur_str = cur_str.substr(pos+1);

}
