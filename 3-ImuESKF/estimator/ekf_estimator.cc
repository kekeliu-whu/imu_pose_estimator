#include "ekf_estimator.h"
#include "common/common.h"
#include "common/utils.h"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <ceres/ceres.h>

namespace {
    Eigen::Matrix<double, 3, 4> GetMeasureEquationJacobian(const Quaterniond &q, const Vec3d &g) {
        auto f = CostFunctor::Create(g);

        std::vector<const double *> pb;
        pb.push_back(q.coeffs().data());

        double null_placeholder[100];

        std::vector<double *> jacobians;
        Eigen::Matrix<double, 3, 4> J;
        for (int i = 0; i < J.rows(); ++i) {
            jacobians.push_back(&J(i, 0));
        }

        // This code gets timed
        f->Evaluate(pb.data(), null_placeholder, jacobians.data());

        return J;
    }
}

Quaterniond EkfEstimator::EstimatePose(double timestamp, const Vec3d &ang, const Vec3d &acc) {
    if (!this->init_ok) {
        this->pose = Quaterniond::Identity();
        this->init_ok = true;
        this->last_timestamp = timestamp;
        this->P.setZero();
        this->g_w = acc;
        return this->pose;
    }

    auto delta_theta = (timestamp - last_timestamp) * ang;
    auto dq = Quaterniond(1, delta_theta.x() / 2, delta_theta.y() / 2, delta_theta.z());
    Eigen::Matrix<double,4,4> F = Qr(dq);

    // predict
    // todo kk add progress noise
    Quaterniond x_prior = this->pose * dq;
    x_prior.normalize();
    Eigen::Matrix<double,4,4> P_prior = F * this->P * F.transpose();

    // update
    // todo kk add measurement noise
    // todo kk do not use inverse()
    Eigen::Matrix<double, 3, 4> H = GetMeasureEquationJacobian(x_prior, this->g_w);
    Eigen::Matrix<double, 4, 3> K = P_prior * H.transpose() * (H * P_prior * H.transpose()).inverse();
    Eigen::Matrix<double, 4, 1> x_posterior = x_prior.coeffs() + K * (acc - x_prior.inverse() * this->g_w);
    Eigen::Matrix<double, 4, 4> P_posterior = (Eigen::Matrix4d::Identity() - K * H) * P_prior;

    this->pose = x_posterior;
    this->pose.normalize();
    this->P = P_posterior;

    return this->pose;
}