#include "common/utils.h"
#include "ekf_estimator.h"

#include <ceres/ceres.h>
#include <gtest/gtest.h>

#include <benchmark/benchmark.h>

static void BM_SomeFunction(benchmark::State &state) {
  // Perform setup here
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

  for (auto _ : state) {
    // This code gets timed
    f->Evaluate(pb.data(), buffer, jacobians.data());
  }
}
// Register the function as a benchmark
BENCHMARK(BM_SomeFunction);
// Run the benchmark
BENCHMARK_MAIN();

/*

Result:
1. Turn on "-march=native"
  Run on (8 X 4100 MHz CPU s)
  CPU Caches:
      L1 Data 32K (x4)
      L1 Instruction 32K (x4)
      L2 Unified 256K (x4)
      L3 Unified 8192K (x1)
  Load Average: 1.80, 1.64, 1.50
  ***WARNING*** CPU scaling is enabled, the benchmark real time measurements may
be noisy and will incur extra overhead.
  ----------------------------------------------------------
  Benchmark                Time             CPU   Iterations
  ----------------------------------------------------------
  BM_SomeFunction       34.3 ns         34.3 ns     20594404

2. Turn off "-march=native"
  Run on (8 X 4100 MHz CPU s)
  CPU Caches:
    L1 Data 32K (x4)
    L1 Instruction 32K (x4)
    L2 Unified 256K (x4)
    L3 Unified 8192K (x1)
  Load Average: 1.95, 1.81, 1.59
  ***WARNING*** CPU scaling is enabled, the benchmark real time measurements may
be noisy and will incur extra overhead.
  ----------------------------------------------------------
  Benchmark                Time             CPU   Iterations
  ----------------------------------------------------------
  BM_SomeFunction       53.2 ns         53.2 ns     11031242


 */
