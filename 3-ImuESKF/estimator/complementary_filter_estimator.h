#ifndef IMU_ESKF_COMPLEMENTARY_FILTER_ESTIMATOR_H
#define IMU_ESKF_COMPLEMENTARY_FILTER_ESTIMATOR_H

#include "estimator.h"

class ComplementaryFilterEstimator : public Estimator {
public:
  Quaterniond EstimatePose(double timestamp, const Vec3d &angular_velocity, const Vec3d &linear_acceleration) override;
};

#endif // IMU_ESKF_COMPLEMENTARY_FILTER_ESTIMATOR_H
