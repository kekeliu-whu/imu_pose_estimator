#ifndef IMU_ESKF_ESTIMATOR_H
#define IMU_ESKF_ESTIMATOR_H

#include "../common/common.h"

class Estimator {
public:
    Estimator() = default;

    virtual ~Estimator() = default;

    virtual Quaterniond EstimatePose(double timestamp, const Vec3d &ang, const Vec3d &acc) = 0;


public:
    double get_last_timestamp() const {
        return last_timestamp;
    }

    bool get_init_ok() const {
        return init_ok;
    }


protected:
    bool init_ok;
    Quaterniond pose;
    double last_timestamp;
};

#endif //IMU_ESKF_ESTIMATOR_H
