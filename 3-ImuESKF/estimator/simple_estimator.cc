#include "simple_estimator.h"

Quaterniond SimpleEstimator::EstimatePose(double timestamp, const Vec3d &ang, const Vec3d &acc) {
  double dt = timestamp - this->last_timestamp;
  this->last_timestamp = timestamp;
  if (!this->init_ok) {
    this->InitPoseByGravity(acc);
    this->init_ok = true;
    return this->pose;
  }

  auto delta_theta = dt * ang;
  auto dq = Quaterniond(1, delta_theta.x() / 2, delta_theta.y() / 2, delta_theta.z() / 2);
  this->pose *= dq;
  this->pose.normalize();

  return this->pose;
}
