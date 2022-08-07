#ifndef IMU_ESKF_UTILS_H
#define IMU_ESKF_UTILS_H

#include "common/common.h"
#include <Eigen/src/Core/Matrix.h>

template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &v3d) {
  typedef typename Derived::Scalar Scalar_t;

  const double kAngleEpisode = 1e-6;
  double theta = v3d.norm();
  double half_theta = 0.5 * theta;

  double imag_factor;
  double real_factor = cos(half_theta);
  if (theta < kAngleEpisode) {
    double theta_sq = theta * theta;
    double theta_po4 = theta_sq * theta_sq;
    // taylor expansion of sin(t/2)/t, visit https://www.wolframalpha.com/input/?i=sin%28t%2F2%29%2Ft for reference.
    imag_factor = 0.5 - (1 / 48.) * theta_sq + (1 / 3840.) * theta_po4;
  } else {
    double sin_half_theta = sin(half_theta);
    imag_factor = sin_half_theta / theta;
  }

  // return {cos(|t|/2), sin(|t|/2)/|t|*t}
  return Eigen::Quaterniond(real_factor, imag_factor * v3d.x(), imag_factor * v3d.y(), imag_factor * v3d.z())
      .cast<Scalar_t>();
}

inline Eigen::Matrix3d Skew(const Vec3d &t) {
    Eigen::Matrix3d t_hat;
    t_hat << 0, -t(2), t(1), t(2), 0, -t(0), -t(1), t(0), 0;
    return t_hat;
}

// eigen quaternion memory layout is xyzw instead of wxyz
template<typename T>
Eigen::Matrix<T, 4, 4> Ql(const Eigen::Quaternion<T> &q) {
    Eigen::Matrix<T, 4, 4> ans;
    ans.template block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * q.w() + Skew(q.vec());
    ans.template block<3, 1>(0, 3) = q.vec();
    ans.template block<1, 3>(3, 0) = -q.vec();
    ans(3, 3) = q.w();
    return ans;
}

template<typename T>
Eigen::Matrix<T, 4, 4> Qr(const Eigen::Quaternion<T> &q) {
    Eigen::Matrix<T, 4, 4> ans;
    ans.template block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * q.w() - Skew(q.vec());
    ans.template block<3, 1>(0, 3) = q.vec();
    ans.template block<1, 3>(3, 0) = -q.vec();
    ans(3, 3) = q.w();
    return ans;
}

#endif // IMU_ESKF_UTILS_H
