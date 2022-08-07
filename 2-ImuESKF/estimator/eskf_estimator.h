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

  // from imu_util result
  const double gyr_n = 1.2138209013030823e-03;
  const double gyr_w = 2.0725450906070991e-05;
  const double acc_n = 1.9881978694148459e-02;
  // TODO fine-tune progress noise and measurement noise here.
  Eigen::Matrix<double, 6, 6> R_;
  Eigen::Matrix<double, 3, 3> Q_;
};

#endif // IMU_ESKF_ESKF_ESTIMATOR_H
