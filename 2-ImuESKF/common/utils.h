#ifndef IMU_ESKF_UTILS_H
#define IMU_ESKF_UTILS_H

#include "common/common.h"
#include <Eigen/src/Core/Matrix.h>

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
