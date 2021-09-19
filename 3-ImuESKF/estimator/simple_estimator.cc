#include "simple_estimator.h"

Quaterniond SimpleEstimator::EstimatePose(double timestamp, const Vec3d &angular_velocity,
                                          const Vec3d &linear_acceleration) {
  double dt = timestamp - this->last_timestamp;
  this->last_timestamp = timestamp;
  if (!this->init_ok) {
    this->InitPoseByGravity(linear_acceleration);
    this->init_ok = true;
    return this->pose;
  }

  auto delta_theta = dt * angular_velocity;
  auto dq = Quaterniond(1, delta_theta.x() / 2, delta_theta.y() / 2, delta_theta.z() / 2);
  this->pose *= dq;
  this->pose.normalize();

  return this->pose;
}
