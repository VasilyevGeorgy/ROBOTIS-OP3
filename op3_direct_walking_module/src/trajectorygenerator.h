#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

#define YOFFSET 0.035
#define PELVISOFFSET 0.005
#define MAXHEIGHT 0.3697
#define MANAGERHEIGHT 0.1547

namespace  robotis_op{

class TrajectoryGenerator
{
public:
  TrajectoryGenerator();
  virtual ~TrajectoryGenerator();

  enum leg_type{
    RIGHT,
    LEFT
  };

  void initialize(Eigen::Matrix4d pel_pose, Eigen::Matrix4d rf_pose, Eigen::Matrix4d lf_pose);

  //void pevlisTranslation(double operating_rate, double duration, Eigen::Matrix4d goal_pose);

  Eigen::Matrix4d comTranslation(leg_type leg, Eigen::Matrix4d goal_pose);
  Eigen::Matrix4d footTranslation(leg_type leg, Eigen::Matrix4d goal_pose);

  //void generateTrajectory(double operating_rate);
  //void getTrajectory(leg_type leg, std::vector <Eigen::Matrix4d> &traj_vec);

  Eigen::Matrix4d getInverseTransform(Eigen::Matrix4d transf); // get inverse transformation matrix
  Eigen::Vector3d getRPY(Eigen::Matrix3d rot_m);
  Eigen::Vector3d getRPY(Eigen::Matrix4d transf_m);
  Eigen::Matrix3d rpyToRotM(Eigen::Vector3d rpy);
  Eigen::Matrix4d getDiffTransf(Eigen::Matrix4d init_pose, Eigen::Matrix4d goal_pose, unsigned int num_steps);
  Eigen::Vector3d getDiffRPY(Eigen::Matrix4d init_pose, Eigen::Matrix4d goal_pose, unsigned int num_steps);

private:

  //void intitStep(leg_type leg, double operating_rate);
  //void makeStep(leg_type leg, double operating_rate);
  Eigen::Matrix4d pelvis_pose;
  Eigen::Matrix4d rfoot_pose;
  Eigen::Matrix4d lfoot_pose;

  std::vector <Eigen::Matrix4d> rleg_traj;
  std::vector <Eigen::Matrix4d> lleg_traj;

};

}

#endif // TRAJECTORYGENERATOR_H
