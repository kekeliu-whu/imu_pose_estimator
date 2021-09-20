#ifndef IMU_ESKF_SIMPLE_ESTIMATOR_H
#define IMU_ESKF_SIMPLE_ESTIMATOR_H

#include "estimator.h"

class SimpleEstimator : public Estimator {
public:
  SimpleEstimator() = default;

  Quaterniond EstimatePose(double timestamp, const Vec3d &angular_velocity, const Vec3d &linear_acceleration) override;
};

#endif // IMU_ESKF_SIMPLE_ESTIMATOR_H
