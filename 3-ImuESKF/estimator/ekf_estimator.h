#ifndef IMU_ESKF_EKF_ESTIMATOR_H
#define IMU_ESKF_EKF_ESTIMATOR_H

#include "estimator.h"
#include <ceres/ceres.h>

#include <utility>

class EkfEstimator : public Estimator {
public:
  Quaterniond EstimatePose(double timestamp, const Vec3d &angular_velocity, const Vec3d &linear_acceleration) override;

private:
  Eigen::Matrix4d P;
  Vec3d g_w;
};

struct CostFunctor {
private:
  explicit CostFunctor(Vec3d g) : g_(std::move(g)) {}

public:
  template <typename T> bool operator()(const T *const x, T *residual) const {
    Eigen::Quaternion<T> q{x};
    Eigen::Map<Eigen::Matrix<T, 3, 1>>{residual} = q * g_.template cast<T>();
    return true;
  }

  static ceres::CostFunction *Create(const Vec3d &g) {
    return new ceres::AutoDiffCostFunction<CostFunctor, 3, 4>(new CostFunctor(g));
  }

private:
  Vec3d g_;
};

#endif // IMU_ESKF_EKF_ESTIMATOR_H
