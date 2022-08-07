#include "simple_estimator.h"
#include "common/utils.h"

Quaterniond SimpleEstimator::EstimatePose(double timestamp, const Vec3d &angular_velocity,
                                          const Vec3d &linear_acceleration) {
  double dt = timestamp - this->last_timestamp;
  this->last_timestamp = timestamp;
  if (!this->init_ok) {
    this->InitPoseByGravity(linear_acceleration);
    return this->pose;
  }

  auto delta_theta = dt * angular_velocity;
  auto dq = deltaQ(delta_theta);
  this->pose *= dq;
  this->pose.normalize();

  return this->pose;
}
