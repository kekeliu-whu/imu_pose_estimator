#include "complementary_filter_estimator.h"
#include "common/common.h"
#include "common/utils.h"

Quaterniond ComplementaryFilterEstimator::EstimatePose(double timestamp, const Vec3d &angular_velocity,
                                                       const Vec3d &linear_acceleration) {
  // normalize acc_norm and consider it as gravity vector directly
  auto acc_norm = linear_acceleration.normalized();

  double dt = timestamp - this->last_timestamp;
  this->last_timestamp = timestamp;
  if (!this->init_ok) {
    this->InitPoseByGravity(acc_norm);
    this->init_ok = true;
    return this->pose;
  }

  auto delta_theta = dt * angular_velocity;
  auto dq = Quaterniond(1, delta_theta.x() / 2, delta_theta.y() / 2, delta_theta.z() / 2);
  Quaterniond pose_prior = this->pose * dq;
  pose_prior.normalize();

  auto prior_dq = Quaterniond::FromTwoVectors(pose_prior * acc_norm, Vec3d::UnitZ());
  auto weighted_prior_dq = Quaterniond::Identity().slerp(0.02, prior_dq);

  this->pose = weighted_prior_dq * pose_prior;

  return this->pose;
}
