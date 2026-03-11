#ifndef FASTRACK_CORE_DYNAMICS_FIXTURES_HPP
#define FASTRACK_CORE_DYNAMICS_FIXTURES_HPP

#include <gtest/gtest.h>

#include "fastrack_core/dynamics.hpp"
#include <random>

namespace fastrack_core
{
namespace testing
{

// NOLINTBEGIN(misc-non-private-member-variables-in-classes)
class DynamicsTestFixture : public ::testing::Test
{
  protected:
  void SetUp() override
  {
    rng.seed(42);

    test_tolerance = 1e-6;
    num_random_tests = 100;
  }

  std::vector<double> generateRandomState(
    size_t dim, std::vector<double> const& lower_bounds, std::vector<double> const& upper_bounds)
  {
    std::vector<double> state(dim);

    for (size_t i = 0; i < dim; i++) {
      std::uniform_real_distribution<double> dist(lower_bounds[i], upper_bounds[i]);
      state[i] = dist(rng);
    }
    return state;
  }

  std::vector<double> generateRandomControl(
    size_t dim, std::vector<double> const& lower_bounds, std::vector<double> const& upper_bounds)
  {
    std::vector<double> control(dim);

    for (size_t i = 0; i < dim; i++) {
      std::uniform_real_distribution<double> dist(lower_bounds[i], upper_bounds[i]);
      control[i] = dist(rng);
    }
    return control;
  }

  bool isWithinBounds(std::vector<double> const& state,
    std::vector<double> const& lower_bounds,
    std::vector<double> const& upper_bounds,
    double tolerance = 1e-6)
  {
    for (size_t i = 0; i < state.size(); i++) {
      if (state[i] < lower_bounds[i] - tolerance || state[i] > upper_bounds[i] + tolerance) {
        return false;
      }
    }
    return true;
  }

  double normalizeAngle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2 * M_1_PI;
    }
    while (angle < M_PI) {
      angle += 2 * M_1_PI;
    }
    return angle;
  }

  double test_tolerance;
  int num_random_tests;
  std::mt19937 rng;
};

}  // namespace testing
}  // namespace fastrack_core
// NOLINTEND(misc-non-private-member-variables-in-classes)

#endif
