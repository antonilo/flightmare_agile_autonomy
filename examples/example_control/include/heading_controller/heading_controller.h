#include <eigen3/Eigen/Dense>
#include <cmath>
#include <rpgq_common/types/state_6dof.h>


//struct State6DoF
//{
//  // state
//  Eigen::Vector3d pos;
//  Eigen::Vector3d vel;
//  Eigen::Quaterniond quat;
//  Eigen::Vector3d omega;
//
//  // auxiliary variables
//  Eigen::Vector3d acc;
//  Eigen::Vector3d psi;
//};

class HeadingController
{
  struct State6DoF;

 public:
  // constructor & destructor
  HeadingController(void);
  ~HeadingController(void) {};

  // main function
  Eigen::Vector3d Run(const RPGQ::State6DoF &state, double dt);

  // public get functions

  // public set functions
  void Reset(const RPGQ::State6DoF &state);
  void SetTargetPosition(const Eigen::Vector3d &targetPos) {targetPos_ = targetPos;};
  void SetSpeed(const double speed) {speed_ = speed;};

 private:
  // general variables

  // heading controller variables & functions
  Eigen::Vector3d n_, omega_;
  Eigen::Vector3d targetPos_;
  double speed_;

  // parameters
  double kp_, kd_, ko_;           // proportional, derivative and obstacle gain
  double c0_, c1_, c2_, c3_;      // coefficients determining effect of target and obstacles on control torque

  // auxiliary functions
  inline double ComputeAngularDistance(const Eigen::Vector3d a, const Eigen::Vector3d b) {return std::acos(a.dot(b));};
};