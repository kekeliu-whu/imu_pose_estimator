#include "simple_estimator.h"

Quaterniond SimpleEstimator::EstimatePose(double timestamp, const Vec3d &ang, const Vec3d &acc) {
    if (!this->init_ok) {
        this->pose = Quaterniond::Identity();
        this->init_ok = true;
        this->last_timestamp = timestamp;
        return this->pose;
    }
    double delta_t = timestamp - this->last_timestamp;
    this->pose *= Quaterniond(1, ang[0] * delta_t / 2, ang[1] * delta_t / 2, ang[2] * delta_t / 2);
    this->pose.normalize();

    this->last_timestamp = timestamp;
    return this->pose;
}
