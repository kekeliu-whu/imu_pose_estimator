#include "ekf_estimator.h"
#include "common/utils.h"

#include <ceres/ceres.h>
#include <gtest/gtest.h>

TEST(EkfEstimator, CostFunctor) {
    auto f = CostFunctor::Create({3, 4, 5});

    double q[] = {0, 0, 0, 1};
    std::vector<double *> pb;
    pb.push_back(q);

    double buffer[100];

    std::vector<double *> jacobians;
    Eigen::Matrix<double, 3, 4> J;
    for (int i = 0; i < J.rows(); ++i) {
        jacobians.push_back(&J(i, 0));
    }

    // This code gets timed
    f->Evaluate(pb.data(), buffer, jacobians.data());
}
