#include <stdio.h>
#include "../include/op3_quasistatic_module.h"

namespace robotis_op
{

QuasistaticControlModule::QuasistaticControlModule()
  : control_cycle_msec_(0),
    stop_process_(false),
    is_moving_(false),
    is_updated_(false),
    is_blocked_(false),
    tra_count_(0),
    tra_size_(0),
    check_collision_(true),
    DEBUG(false)
{
  enable_ = false;
  module_name_ = "quasistatic_module";
  control_mode_ = robotis_framework::PositionControl;

  last_msg_time_ = ros::Time::now();
}

QuasistaticControlModule::~QuasistaticControlModule()
{
  queue_thread_.join();
}

void QuasistaticControlModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&QuasistaticControlModule::queueThread, this));

  // init result, joint_id_table
  int joint_index = 0;
  for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
       it != robot->dxls_.end(); it++){

    std::string joint_name = it->first;
    robotis_framework::Dynamixel* dxl_info = it->second;

    result_[joint_name] = new robotis_framework::DynamixelState();
    result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;

    collision_[joint_name] = false;

    using_joint_name_[joint_name] = joint_index++;
  }
  // initialize all coefficiets to zero
  ROS_INFO("joint map length: %lu", result_.size());
  target_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  present_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

  ros::NodeHandle rn;
  // publish topics
  status_msg_pub_ = rn.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);

}

void QuasistaticControlModule::queueThread()
{
  ros::NodeHandle rn;
  ros::CallbackQueue callback_queue;

  rn.setCallbackQueue(&callback_queue);

  ros::Subscriber step_params_sub = rn.subscribe("/robotis/quasistatic/step_params", 1, &QuasistaticControlModule::StepParamsCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while(rn.ok())
    callback_queue.callAvailable(duration);

}

void QuasistaticControlModule::StepParamsCallback(const op3_online_walking_module_msgs::FootStepCommand::ConstPtr &msg)
{
  if (!enable_){

    ROS_INFO_THROTTLE(1, "Quasistatic control module is not enable");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Not Enable");
    return;
  }

  // waiting for updating of present joint states
  int waiting_count = 0;
  while(!is_updated_)
  {
    usleep(control_cycle_msec_ * 1000);
    if(++waiting_count > 100)
    {
      ROS_ERROR("Present joint states aren't updated!");
      return;
    }
  }

  if(!is_moving_){
    // get step parameters from msg
    int num_of_steps = msg->step_num;
    std::string init_leg = msg->start_leg;
    double step_length = msg->step_length;
    double step_duration = msg->step_time;
    double step_clearance = msg->side_length;

    if((num_of_steps > 0) && !init_leg.empty()){

      if(tra_size_ == 0) // (rleg_joint_angles_.size() == 0) || (lleg_joint_angles_.size() == 0)
      {
        op3_quasistatic_locomotion locom;

        // fill step_param structure
        sp.num_of_steps = num_of_steps;
        sp.init_leg = init_leg;
        sp.step_length = step_length;
        sp.step_duration = step_duration;
        sp.step_clearance = step_clearance;
        sp.freq = int(1000/control_cycle_msec_); // control_cycle = 8 msec

        // pelvis pose while locomotion
        KDL::Frame goalPose = KDL::Frame(KDL::Rotation::RPY(0.0,0.0,0.0),
                                         KDL::Vector(0.0, 0.0, 0.3));
        // Calculate angles
        locom.quasiStaticPlaner(goalPose, sp);
        locom.getAnglesVectors(rleg_joint_angles_, lleg_joint_angles_);

        tra_size_ = rleg_joint_angles_.size();
      }
    }
  }
  //else
  //  ROS_INFO("Previous process is still active");

}

void QuasistaticControlModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                  std::map<std::string, double> sensors)
{

  if (enable_ == false)
    return;

  tra_lock_.lock();

  // get joint data from robot
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
       state_it != result_.end(); state_it++)
  {
    std::string joint_name = state_it->first;
    int index = using_joint_name_[joint_name];

    robotis_framework::Dynamixel *_dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      _dxl = dxl_it->second;
    else
      continue;

    present_position_.coeffRef(0, index) = _dxl->dxl_state_->present_position_;
    goal_position_.coeffRef(0, index) = _dxl->dxl_state_->goal_position_;
    result_[joint_name]->goal_position_ = _dxl->dxl_state_->goal_position_;
    collision_[joint_name] = false;
  }

  is_updated_ = true;

  // check to stop
  if (stop_process_)
    stopMoving();
  else
  {
    if (tra_size_ > 0) // process
    {
      if (tra_count_ == 0) // start of steps
        startMoving();

      if (tra_count_ >= tra_size_) // end of steps
        finishMoving();
      else
      { // update goal position
        Eigen::VectorXd cur_val = rleg_joint_angles_.at(tra_count_);

        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //! check numbers' correctness !!!!!!!!!!!
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        goal_position_.coeffRef(0,16) = cur_val(0);
        goal_position_.coeffRef(0,15) = cur_val(1);
        goal_position_.coeffRef(0,14) = cur_val(2);
        goal_position_.coeffRef(0,17) = cur_val(3);
        goal_position_.coeffRef(0,11) = cur_val(4);
        goal_position_.coeffRef(0,12) = cur_val(5);

        cur_val = lleg_joint_angles_.at(tra_count_);
        goal_position_.coeffRef(0,7) = cur_val(0);
        goal_position_.coeffRef(0,6) = cur_val(1);
        goal_position_.coeffRef(0,5) = cur_val(2);
        goal_position_.coeffRef(0,8) = cur_val(3);
        goal_position_.coeffRef(0,2) = cur_val(4);
        goal_position_.coeffRef(0,3) = cur_val(5);

        tra_count_++;
      }
    }
  }

  tra_lock_.unlock();

  usleep(1000);

  // set joint data to robot
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
       state_it != result_.end(); state_it++)
  {
    std::string joint_name = state_it->first;
    int index = using_joint_name_[joint_name];
    double goal_position = goal_position_.coeff(0, index);

    if(collision_[joint_name] == false || check_collision_ == false)
      result_[joint_name]->goal_position_ = goal_position;
  }

}

void QuasistaticControlModule::stop()
{
  tra_lock_.lock();

  if (is_moving_ == true)
    stop_process_ = true;

  tra_lock_.unlock();

}

bool QuasistaticControlModule::isRunning()
{
  return is_moving_;
}

void QuasistaticControlModule::onModuleEnable()
{
  is_updated_ = false;
  is_blocked_ = false;

}

void QuasistaticControlModule::onModuleDisable()
{

}

void QuasistaticControlModule::startMoving()
{
  is_moving_ = true;
}

void QuasistaticControlModule::finishMoving()
{
  // init value
  calc_joint_tra_ = goal_position_;
  tra_size_ = 0;
  tra_count_ = 0;
  is_moving_ = false;

  // log
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Quasisitatic locomotion is finished.");
  ROS_WARN("Quasistatic locomotion is finished");

  setModule("none");

}

void QuasistaticControlModule::stopMoving()
{
  // init value
  calc_joint_tra_ = goal_position_;
  tra_size_ = 0;
  tra_count_ = 0;
  is_moving_ = false;
  stop_process_ = false;

  // log
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Stop Module.");

}

void QuasistaticControlModule::jointTraGeneThread()
{

}

void QuasistaticControlModule::publishStatusMsg(unsigned int type, std::string msg){
  ros::Time now = ros::Time::now();

  if(msg.compare(last_msg_) == 0)
  {
    ros::Duration dur = now - last_msg_time_;
    if(dur.sec < 1)
      return;
  }

  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = now;
  status_msg.type = type;
  status_msg.module_name = "Quasistatic Control";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);

  last_msg_ = msg;
  last_msg_time_ = now;
}

void QuasistaticControlModule::setModule(const std::string &moduleName){

  ros::NodeHandle rn;
  ros::ServiceClient set_joint_module_client = rn.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");

  robotis_controller_msgs::SetModule set_module_srv;
  set_module_srv.request.module_name = moduleName;

  if (set_joint_module_client.call(set_module_srv) == false)
  {
    ROS_WARN("Failed to set module");
    return;
  }

}

}
