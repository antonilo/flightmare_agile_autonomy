#include <heading_controller/heading_controller.h>

#include <cmath>

// constructor
HeadingController::HeadingController(void)
{
  // general variables

  // heading controller variables
  n_.setZero();
  omega_.setZero();
  targetPos_.setZero();
  speed_ = 1.0;

  // parameters
  kp_ = 7.5;
  kd_ = 1.25;
  ko_ = 198.0;
  c0_ = 0.4;
  c1_ = 0.4;
  c2_ = 6.5;
  c3_ = 0.8;
}


// main function
Eigen::Vector3d HeadingController::Run(const RPGQ::State6DoF &state, double dt)
{
  // compute target heading
  Eigen::Vector3d nTarget = targetPos_ - state.pos;
  double dTarget = nTarget.norm();
  nTarget.normalize();

  Eigen::Vector3d u = kp_*speed_*speed_*(std::exp(-c0_*dTarget) + c1_)*nTarget.cross(n_) - kd_*speed_*omega_;

  Eigen::Vector3d ndot = n_.cross(omega_);
  Eigen::Vector3d nddot = ndot.cross(omega_) + n_.cross(u);

  n_ = n_ + ndot*dt + 0.5*nddot*dt*dt;
  n_.normalize();
  omega_ = omega_ + u*dt;
  omega_ = omega_ - omega_.dot(n_)*n_;

  return speed_*n_;
}


// public set functions
void HeadingController::Reset(const RPGQ::State6DoF &state)
{
  n_ = state.vel.normalized();
  omega_.setZero();
}