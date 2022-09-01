#ifndef IMU_ESKF_ESKF_ESTIMATOR_H
#define IMU_ESKF_ESKF_ESTIMATOR_H

#include "estimator.h"

class EskfEstimator : public Estimator {
public:
  EskfEstimator() {
    R_.setZero();
    R_.block<3, 3>(0, 0) = gyr_n * gyr_n * Eigen::Matrix3d::Identity();
    R_.block<3, 3>(3, 3) = gyr_w * gyr_w * Eigen::Matrix3d::Identity();

    Q_.setZero();
    Q_.block<3, 3>(0, 0) = acc_n * acc_n * Eigen::Matrix3d::Identity();
  }

  Quaterniond EstimatePose(double timestamp, const Vec3d &angular_velocity, const Vec3d &linear_acceleration) override;

private:
  Eigen::Matrix<double, 6, 6> P_;
  Vec3d gw_;
  Vec3d bg_;

  // TODO fine-tune progress noise and measurement noise here.
  // result from https://github.com/ori-drs/allan_variance_ros
  const double gyr_n = 9.285066316933984e-05;
  const double gyr_w = 3.231516336970033e-06;
  // *10 because natural accelemeter noise are much bigger when IMU and dynamic
  const double acc_n = 0.0016689945925889259 * 5;

  Eigen::Matrix<double, 6, 6> R_;
  Eigen::Matrix<double, 3, 3> Q_;
};

#endif // IMU_ESKF_ESKF_ESTIMATOR_H
