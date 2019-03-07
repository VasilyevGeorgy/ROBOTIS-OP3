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
    r_min_diff_(0.07),
    l_min_diff_(0.07),
    tra_count_(0),
    tra_size_(0),
    default_moving_time_(0.5),
    default_moving_angle_(30),
    check_collision_(true),
    moving_time_(3.0),
    BASE_INDEX(0),
    HEAD_INDEX(20),
    RIGHT_END_EFFECTOR_INDEX(21),
    RIGHT_ELBOW_INDEX(5),
    LEFT_END_EFFECTOR_INDEX(22),
    LEFT_ELBOW_INDEX(6),
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
  // variable for self_collision_check
  op3_kinematics_ = new OP3KinematicsDynamics(WholeBody);

  // init result, joint_id_table
  int joint_index = 0;
  for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
       it != robot->dxls_.end(); it++)
  {
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

  // print joint names
  //for (std::map<std::string, robotis_framework::DynamixelState*>::iterator it = result_.begin();
  //     it != result_.end(); it++){
  //  std::string joint_name = it->first;
  //  ROS_INFO("%s", joint_name.c_str());
  //}

  // print current module and control_mode
  std::string mod_name = this->getModuleName();
  int cntrl_module = int(this->getControlMode());
  ROS_INFO("Current module name: %s", mod_name.c_str());
  ROS_INFO("Current control mode: %d", cntrl_module);

  // enable = true
  //this->setModuleEnable(true);

  // setting of queue thread
  //queue_thread_ = boost::thread(boost::bind(&QuasistaticControlModule::queueThread, this));

  control_cycle_msec_ = control_cycle_msec;

  ros::NodeHandle ros_node;

  ///* get Param */
  //ros_node.param<double>("/robotis/direct_control/default_moving_time", default_moving_time_, default_moving_time_);
  //ros_node.param<double>("/robotis/direct_control/default_moving_angle", default_moving_angle_, default_moving_angle_);
  //ros_node.param<bool>("/robotis/direct_control/check_collision", check_collision_, check_collision_);

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 0);
}

void QuasistaticControlModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscribe topics */
  ros::Subscriber set_head_joint_sub = ros_node.subscribe("/robotis/direct_control/set_joint_states", 1,
                                                          &QuasistaticControlModule::setJointCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void QuasistaticControlModule::setJointCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  if (enable_ == false)
  {
    ROS_INFO_THROTTLE(1, "Quasistatic control module is not enable.");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Not Enable");
    return;
  }

  // wait for updating of present joint states
  int waiting_count = 0;
  while(is_updated_ == false)
  {
    usleep(control_cycle_msec_ * 1000);
    if(++waiting_count > 100)
    {
      ROS_ERROR("present joint angle is not updated");
      return;
    }
  }

  // set init joint vel, accel
  goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

  if (is_moving_ == true)
  {
    goal_velocity_ = calc_joint_vel_tra_.block(tra_count_, 0, 1, result_.size());
    goal_acceleration_ = calc_joint_accel_tra_.block(tra_count_, 0, 1, result_.size());
  }

  // moving time
  moving_time_ = default_moving_time_;               // default : 0.5 sec

  // set target joint angle
  target_position_ = goal_position_;        // default is goal position

  for (int ix = 0; ix < msg->name.size(); ix++)
  {
    std::string joint_name = msg->name[ix];
    std::map<std::string, int>::iterator joint_it = using_joint_name_.find(joint_name);

    if (joint_it != using_joint_name_.end())
    {
      double target_position = 0.0;
      int joint_index = joint_it->second;

      // set target position
      target_position = msg->position[ix];

      // check angle limit

      // apply target position
      target_position_.coeffRef(0, joint_index) = target_position;

      // set time
      double angle_unit = default_moving_angle_ * M_PI / 180;
      double calc_moving_time = fabs(goal_position_.coeff(0, joint_index) - target_position_.coeff(0, joint_index))
          / angle_unit;
      if (calc_moving_time > moving_time_)
        moving_time_ = calc_moving_time;

      if (DEBUG)
        std::cout << "joint : " << joint_name << ", Index : " << joint_index << ", Angle : " << msg->position[ix]
                     << ", Time : " << moving_time_ << std::endl;

      // in case of moving and collision
      if(collision_[joint_name] == true)
      {
        goal_velocity_.coeffRef(0, joint_index) = 0.0;
        goal_acceleration_.coeffRef(0, joint_index) = 0.0;
      }
    }
  }


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
  if (stop_process_ == true)
  {
    stopMoving();
  }
  else
  {
    // process
    if (tra_size_ != 0)
    {
      // start of steps
      if (tra_count_ == 0)
      {
        startMoving();
      }

      // end of steps
      if (tra_count_ >= tra_size_)
      {
        finishMoving();
      }
      else
      {
        // update goal position
        goal_position_ = calc_joint_tra_.block(tra_count_, 0, 1, result_.size());
        tra_count_ += 1;
      }
    }
  }
  tra_lock_.unlock();

  if(check_collision_ == true)
  {
    // set goal angle and run forward kinematics
    for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
         state_it != result_.end(); state_it++)
    {
      std::string joint_name = state_it->first;
      int index = using_joint_name_[joint_name];
      double goal_position = goal_position_.coeff(0, index);

      LinkData *op3_link = op3_kinematics_->getLinkData(joint_name);
      if(op3_link != NULL)
        op3_link->joint_angle_ = goal_position;
    }

    op3_kinematics_->calcForwardKinematics(0);

    // check self collision
    bool collision_result = checkSelfCollision();
  }
///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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

  return;
}

bool QuasistaticControlModule::isRunning()
{
  return is_moving_;
}

void QuasistaticControlModule::onModuleEnable()
{
  is_updated_ = false;
  is_blocked_ = false;
  r_min_diff_ = 0.07;
  l_min_diff_ = 0.07;
}

void QuasistaticControlModule::onModuleDisable()
{

}

void QuasistaticControlModule::startMoving()
{
  is_moving_ = true;

  // start procedure
}

void QuasistaticControlModule::finishMoving()
{
  // init value
  calc_joint_tra_ = goal_position_;
  tra_size_ = 0;
  tra_count_ = 0;
  is_moving_ = false;

  // log
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Head movement is finished.");

  if (DEBUG)
    std::cout << "Trajectory End" << std::endl;
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
  //tra_lock_.lock();
  //tra_lock_.unlock();
}

void QuasistaticControlModule::publishStatusMsg(unsigned int type, std::string msg)
{
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
}
