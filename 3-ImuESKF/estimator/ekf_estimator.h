#ifndef IMU_ESKF_EKF_ESTIMATOR_H
#define IMU_ESKF_EKF_ESTIMATOR_H


#include "estimator.h"

class EkfEstimator : public Estimator {
public:
    Quaterniond EstimatePose(double timestamp, const Vec3d &ang, const Vec3d &acc) override;

};


#endif //IMU_ESKF_EKF_ESTIMATOR_H
