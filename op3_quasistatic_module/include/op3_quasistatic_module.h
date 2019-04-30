#ifndef QUASISTATIC_CONTROL_MODULE_H_
#define QUASISTATIC_CONTROL_MODULE_H_

#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>

#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"

#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"

#include "../../op3_quasistatic_locomotion/src/op3_quasistatic_locomotion.h"
#include "op3_online_walking_module_msgs/FootStepCommand.h"

namespace robotis_op
{

class QuasistaticControlModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<QuasistaticControlModule>
{
 public:
  QuasistaticControlModule();
  virtual ~QuasistaticControlModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  void onModuleEnable();
  void onModuleDisable();

 private:
   enum TraIndex
   {
     Position = 0,
     Velocity = 1,
     Acceleration = 2,
     Count
   };

  /* ROS Topic Callback Functions */
  void StepParamsCallback(const op3_online_walking_module_msgs::FootStepCommand::ConstPtr &msg);
  void keyboardContolCallback(const std_msgs::Int32::ConstPtr &msg);

  void queueThread();
  void jointTraGeneThread();

  void startMoving();
  void finishMoving();
  void stopMoving();

  void publishStatusMsg(unsigned int type, std::string msg);

  std::map<std::string, bool> collision_;

  bool checkSelfCollision();

  double default_moving_time_;
  double default_moving_angle_;
  bool check_collision_;

  int control_cycle_msec_;
  boost::thread queue_thread_;
  boost::thread *tra_gene_thread_;
  boost::mutex tra_lock_;

  ros::Publisher status_msg_pub_;
  ros::Publisher phase_pub_;

  const bool DEBUG;
  bool stop_process_;
  bool is_moving_;
  bool is_updated_;
  bool is_blocked_;
  bool will_be_collision_;
  int tra_count_, tra_size_;
  double moving_time_;
  int keyboard_control;
  int pres_phase;
  int prev_phase;

  Eigen::MatrixXd target_position_;
  Eigen::MatrixXd present_position_;
  Eigen::MatrixXd goal_position_;
  Eigen::MatrixXd goal_velocity_;
  Eigen::MatrixXd goal_acceleration_;
  Eigen::MatrixXd calc_joint_tra_;
  Eigen::MatrixXd calc_joint_vel_tra_;
  Eigen::MatrixXd calc_joint_accel_tra_;

  std::map<std::string, int> using_joint_name_;
  std::map<int, double> max_angle_;
  std::map<int, double> min_angle_;

  ros::Time last_msg_time_;
  std::string last_msg_;

  OP3KinematicsDynamics *op3_kinematics_;

  std::vector<Eigen::VectorXd> rleg_joint_angles_;
  std::vector<Eigen::VectorXd> lleg_joint_angles_;
  std::vector<int> phases_;

  void setModule(const std::string &moduleName);
  op3_quasistatic_locomotion::stepParam sp;

};

}

#endif /* HEAD_CONTROL_MODULE_H_ */
