#include "utils.h"
#include <gtest/gtest.h>

const double kEpsilon = 1e-8;

TEST(UtilsTest, Ql) {
    auto q1 = Quaterniond::UnitRandom();
    auto q2 = Quaterniond::UnitRandom();

    Eigen::Vector4d ans1 = Ql(q1) * q2.coeffs();
    Eigen::Vector4d ans2 = (q1 * q2).coeffs();
    EXPECT_LT((ans1 - ans2).norm(), kEpsilon);
}

TEST(UtilsTest, Qr) {
    auto q1 = Quaterniond::UnitRandom();
    auto q2 = Quaterniond::UnitRandom();

    Eigen::Vector4d ans1 = Qr(q2) * q1.coeffs();
    Eigen::Vector4d ans2 = (q1 * q2).coeffs();
    EXPECT_LT((ans1 - ans2).norm(), kEpsilon);
}

//int main(int argc, char **argv) {
//    ::testing::InitGoogleTest(&argc, argv);
//    return RUN_ALL_TESTS();
//}
