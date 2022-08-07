#include "ekf_estimator.h"
#include "common/common.h"
#include "common/utils.h"

#include <ceres/ceres.h>

namespace {
Eigen::Matrix<double, 3, 4> GetMeasureEquationJacobian(const Quaterniond &q, const Vec3d &g) {
  auto f = CostFunctor::Create(g);

  std::vector<const double *> pb;
  pb.push_back(q.coeffs().data());

  double null_placeholder[100];

  std::vector<double *> jacobians;
  Eigen::Matrix<double, 3, 4> J;
  for (int i = 0; i < J.rows(); ++i) {
    jacobians.push_back(&J(i, 0));
  }

  // This code gets timed
  f->Evaluate(pb.data(), null_placeholder, jacobians.data());

  return J;
}
} // namespace

Quaterniond EkfEstimator::EstimatePose(double timestamp, const Vec3d &angular_velocity,
                                       const Vec3d &linear_acceleration) {
  // normalize acc and consider it as gravity vector directly
  auto acc_norm = linear_acceleration.normalized();
  this->g_w = acc_norm;

  double dt = timestamp - this->last_timestamp;
  this->last_timestamp = timestamp;
  if (!this->init_ok) {
    this->InitPoseByGravity(acc_norm);
    this->P.setZero();
    return this->pose;
  }

  auto delta_theta = dt * angular_velocity;
  auto dq = Quaterniond(1, delta_theta.x() / 2, delta_theta.y() / 2, delta_theta.z() / 2);
  Eigen::Matrix<double, 4, 4> F = Qr(dq);

  // predict
  Quaterniond x_prior = this->pose * dq;
  x_prior.normalize();
  // std::cout << "A: " << x_prior.coeffs().transpose() << std::endl;
  // std::cout << "B: " << F * this->pose.coeffs() << std::endl;
  Eigen::Matrix<double, 4, 4> P_prior = F * this->P * F.transpose() + R;

  // update
  // todo kk do not use inverse()
  Eigen::Matrix<double, 3, 4> H = GetMeasureEquationJacobian(x_prior, this->g_w);
  Eigen::Matrix<double, 4, 3> K = P_prior * H.transpose() * (H * P_prior * H.transpose() + Q).inverse();
  Eigen::Matrix<double, 4, 1> x_posterior =
      x_prior.coeffs() + K * (Eigen::Vector3d::UnitZ() - x_prior.inverse() * this->g_w);
  Eigen::Matrix<double, 4, 4> P_posterior = (Eigen::Matrix4d::Identity() - K * H) * P_prior;

  this->pose = x_posterior;
  this->pose.normalize();
  this->P = P_posterior;

  return this->pose;
}
