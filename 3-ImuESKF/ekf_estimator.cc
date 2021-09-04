#include "ekf_estimator.h"

Quaterniond EkfEstimator::EstimatePose(double timestamp, const Vec3d &ang, const Vec3d &acc) {
    if (!this->init_ok) {
        this->pose = Quaterniond::Identity();
        this->init_ok = true;
        this->last_timestamp = timestamp;
        return this->pose;
    }
    return this->pose;
}
