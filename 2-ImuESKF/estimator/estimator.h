#ifndef IMU_ESKF_ESTIMATOR_H
#define IMU_ESKF_ESTIMATOR_H

#include "common/common.h"

class Estimator {
public:
  Estimator() : init_ok(false) {}

  virtual ~Estimator() = default;

  virtual Quaterniond EstimatePose(double timestamp, const Vec3d &angular_velocity,
                                   const Vec3d &linear_acceleration) = 0;

public:
  double get_last_timestamp() const { return last_timestamp; }

  bool get_init_ok() const { return init_ok; }

  void InitPoseByGravity(const Vec3d &g) {
    // warning: FromTwoVectors() should be used as a static function
    this->pose = Quaterniond::FromTwoVectors(g, Vec3d::UnitZ());
    this->init_ok = true;
  }

protected:
  bool init_ok;
  Quaterniond pose;
  double last_timestamp;
};

#endif // IMU_ESKF_ESTIMATOR_H
