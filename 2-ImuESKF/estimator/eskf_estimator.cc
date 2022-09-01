#include "eskf_estimator.h"
#include "common/common.h"
#include "common/utils.h"

#include <ceres/ceres.h>

namespace {} // namespace

Quaterniond EskfEstimator::EstimatePose(double timestamp, const Vec3d &angular_velocity,
                                        const Vec3d &linear_acceleration) {
  // normalize acc and consider it as gravity vector directly
  auto acc_norm = linear_acceleration.normalized();
  this->gw_ = acc_norm;

  double dt = timestamp - this->last_timestamp;
  this->last_timestamp = timestamp;
  if (!this->init_ok) {
    this->InitPoseByGravity(acc_norm);
    this->P_.setZero();
    return this->pose;
  }

  auto delta_theta = dt * angular_velocity;
  auto dq = deltaQ(delta_theta);

  // predict
  Quaterniond x_prior = this->pose * dq;
  x_prior.normalize();
  Eigen::Matrix<double, 6, 6> J;
  J.block<3, 3>(0, 0) = -Skew(angular_velocity - bg_) * dt + Eigen::Matrix3d::Identity();
  J.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity() * dt;
  J.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
  J.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

  auto R = R_;
  R.block<3, 3>(0, 0) /= dt;
  R.block<3, 3>(3, 3) *= dt;
  Eigen::Matrix<double, 6, 6> P_prior = J * this->P_ * J.transpose() + R * dt * dt;

  // update
  Eigen::Matrix<double, 3, 6> H;
  H.block<3, 3>(0, 0) = Skew(pose.inverse() * Eigen::Vector3d::UnitZ());
  H.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();

  auto Q = Q_;
  Q.block<3, 3>(0, 0) /= dt;
  Eigen::Matrix<double, 6, 3> K = P_prior * H.transpose() * (H * P_prior * H.transpose() + Q).inverse();
  Eigen::Matrix<double, 6, 1> x_posterior = K * (Eigen::Vector3d::UnitZ() - x_prior.inverse() * this->gw_);
  Eigen::Matrix<double, 6, 6> P_posterior = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * P_prior;

  this->pose = x_prior * deltaQ(x_posterior.head<3>());
  this->bg_ = this->bg_ + x_posterior.tail<3>();
  this->P_ = P_posterior;

  // double slope_degree = (Eigen::Vector3d::UnitZ() - x_prior.inverse() * this->gw_).norm() * 180 / M_PI;
  // PRINT_MATRIX(slope_degree);
  // PRINT_MATRIX(x_posterior);
  // PRINT_MATRIX(P_);
  // PRINT_MATRIX(P_prior);
  // PRINT_MATRIX(K);
  // PRINT_MATRIX(x_posterior);
  // PRINT_MATRIX(P_posterior);
  PRINT_MATRIX(this->bg_);
  std::cout << "\n\n";

  return this->pose;
}
